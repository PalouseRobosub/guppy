# guppy_tasks

This package contains high level task execution code written in Python. It publishes velocity information to `/cmd_vel/task` and is called from the nav decision tree.

Currently, here is the breakdown on the tasks we are planning to do:

Our main hope for Guppy in RoboSub 2026 was to pre-qualify for competition, which we have successfully accomplished! For this to happen, we had to break the process down into the following sub-goals:
-	Research and design an AUV concept.
-	Manufacture the mechanical components of the AUV.
-	Design and assemble an electrical harness.
-	Develop and write manual control software.
-	Develop and write autonomous control software.
-	Test and iterate on the design.

As we were on a very tight timeline, this was our only primary goal for competition: *have a working submarine that can successfully navigate autonomously under the water*. Anything above and beyond that was fantastic, but not our primary objective. We planned to leave school at the beginning of May with a solid platform for future development, and one that could hopefully get us some points in competition. 

As for concrete task goals in competition, we planned for the tasks that primarily focused on mobility (rather than additional subsystems such as an arm). These were the starting gate (Begin Assessment), the slalom task (Avoid Debris), surfacing inside the octagon (Resupply), Return Home, and (pre) qualification. 

We are mainly aiming for "Core" points for these tasks according to the task capability matrices, although we haven't been able to complete a full points-based analysis because the points for the tasks have not yet been released. To complete some tasks, including the “Advanced” tracks, we need more testing and integration of our vision systems. Currently, most of the tasks we can complete are based on dead-reckoning, however over the summer we hope to finalize the vision system.

Despite the focus on mobility-only tasks, our club had several teams developing additional capabilities. These include torpedoes (Deploy) and a claw (Recon and Resupply). However, due to time constraints, these features will likely have to be pushed to next year. Our rubber-band-powered torpedo prototype has shown extreme potential, and there is a chance we will be able to attempt it at competition. Our claw is also quite far along in its design but needs a bit more development time due to task objects not matching our size expectations after our initial design.

As we plan to re-use Guppy in the future, we will have plenty of time to develop these capabilities over the next year. We plan to move to a much more rapid development/test cycle next year, which will allow us to compete in 2027 with torpedoes and a claw. Our current task capabilities are listed below:

<img width="753" height="258" alt="image" src="https://github.com/user-attachments/assets/19a3a2f3-c05c-4ef5-8f38-4d6f5dc5a04f" />
