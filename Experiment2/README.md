# RE510

# Title : Introduction to ROS

Learn the basic concept of the Multi node communication in ROS.
Implement a 3-6-9 game with three nodes.

---

### Experiment details and scoring

Create a code that

- **Node 1(Number Generator Node):**
  Publishes sequential integers starting from 1, incrementing by 1 at a frequency of 1 Hz.

- **Node 2(Game Logic Node):** Receives numbers from Node 1 and analyzes them. It determines whether each number complies with the 3-6-9 game rules (i.e., if the number contains 3, 6, or 9). \\ Based on this analysis, it publishes a custom message that includes a string indicating "Clap" or "Pass," alongside the respective number.

- **Node 3(Game Output Node):** Subscribes to the messages from Node 2. It prints statements like “3 is a clap” or “5 is a pass”.
- **Finally, create a Launch file that only the node3 result prints out on terminal.**

---

### Source codes

No Source codes provided.

After changing code, run

catkin_make

source devel/setup.bash
