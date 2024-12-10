# Getting Started

ROS has always been a somewhat large project to learn about.
This page contains a set of suggested readings - and a suggested order - to get started:

 * [Installation](https://docs.ros.org/en/jazzy/Installation.html) - The first thing to do is install ROS 2.
   Don't be fancy - just install from Debian packages on the proper version of Ubuntu on an AMD64 architecture computer
   (I know you are excited to try out that Raspberry Pi you got for cheap - but save those headaches for another day).
 * [Beginner Concepts](https://docs.ros.org/en/jazzy/Concepts/Basic.html) - Eventually you'll be ranting some day that
   `ROS is just pub-sub` - but first you need to know what that means. These concept pages cover everything at a high
   level before you actually just jump into doing stuff.
 * [Beginner CLI Tools](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools.html) - Yes, these are boring.
   But also yes, you need to know this stuff. Definitely pay attention to the `nodes` and `topics` stuff - you can always
   come back later to learn about `services` and `actions`.
 * [Beginner Client Libraries](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries.html) - Now we actually
   start writing some code. You don't have to do all of these - pick your language (C++ or Python) and focus on one language
   at a time.
 * [Concepts - TF2](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Tf2.html) and
   [Intermediate - TF2](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/Tf2-Main.html) - Assuming you want to
   actually play with real robots, you will want to learn TF2. This is used all over ROS for handling coordinate frame
   transformations.
 * [Intermediate - Launch](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Launch-Main.html) - Now you've written
   some code - as you start to build bigger systems, you'll want to use ROS launch.
 * [Intermediate - Actions](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Writing-an-Action-Server-Client/Cpp.html) - Now
   would be a good time to learn about actions - if you skipped the "Beginner Concepts" and "Beginner CLI Tools" sections on
   actions, then go back and read them first.
 * Finally, go back and read the whole [Concepts](https://docs.ros.org/en/jazzy/Concepts.html) section of the documentation.
   We looked at some of it early on, but there is an intermediate and advanced section as well.

With this set of tutorials under your belt, you should have a pretty good handle on the main tools
available in ROS. And with that understanding, it should be easier to start jumping around in the rest of the
documentation and learning about [writing](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Writing-a-Composable-Node.html)
and [using composable nodes](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Composition.html), or
[how to write tests in ROS 2](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Testing/Testing-Main.html).
