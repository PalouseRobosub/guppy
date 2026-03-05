import socket, threading, json, os, rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.logging import get_logger

from guppy_teleop.frontend import build
from guppy_teleop.backend.nodes.state_node import StateNode
from guppy_teleop.backend.nodes.param_node import ParameterNode
from guppy_teleop.backend.registry import Registry

SOCKET_PATH = os.path.join("/tmp/terminal-ipc")

connections = []

logger = get_logger("guppy_terminal.terminal_backend")

def client_handler(connection, registry: Registry):
    registry.sync_client()
    
    try:
        buffer = ""
        while True:
            try:
                data = connection.recv(1024).decode()
            except ConnectionResetError:
                break # client suddenly disconnected
            if not data:
                break

            buffer += data
            while "\n" in buffer:
                line, buffer = buffer.split("\n", 1)

                try:
                    registry.route(json.loads(line))
                except json.JSONDecodeError:
                    logger.error("bad message sent")
    finally:
        if (connection in connections):
            connections.remove(connection)
        connection.close()

def main(args = None):
    build.run()

    rclpy.init(args = args)

    def broadcast(message):
        for connection in connections:
            try:
                connection.sendall((json.dumps(message) + "\n").encode())
            except:
                connections.remove(connection)

    registry = Registry(broadcast)

    nodes = []

    # all nodes go here

    nodes.append(StateNode(registry))
    nodes.append(ParameterNode(registry))

    # end nodes section

    executor = MultiThreadedExecutor()
    for node in nodes:
        registry.register(node)
        executor.add_node(node)
    
    ros_thread = threading.Thread(target = executor.spin, daemon = True)
    ros_thread.start()

    try:
        os.unlink(SOCKET_PATH)
    except OSError:
        if os.path.exists(SOCKET_PATH):
            raise
    
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as server:
        server.bind(SOCKET_PATH)
        server.listen()
        logger.info("guppy_teleop backend listening for connections...")

        try:
            while True:
                connection, client_address = server.accept()
                connections.append(connection)

                logger.info(f"client connected, {len(connections)} total")

                threading.Thread(target=client_handler, args=(connection, registry), daemon=True).start()
        finally:
            for connection in connections:
                connection.close()

            os.unlink(SOCKET_PATH)  

    # cleanup when done...

    executor.shutdown()

    for node in nodes:
        node.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()