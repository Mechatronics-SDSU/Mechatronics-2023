# Some Help With CMake Files and Builds In C++ #

I personally plan on writing my classes in C++ and calling them from the ROS2 nodes to integrate their functionaility into the communication of the Robot.

I know not everyone has used C++ and gets confused about the build process so I figured I'd give a brief overview.

### When It's Easy ###

Since python loves us, it comes with a mostly easy-to-use intrepreter which generates our machine code for us. Unfortunately C/C++ doesn't love us and we have to compile our programs into binaries that the computer can use (it's actually not that bad haha).

This seems simple enough with a one-file type of program. All you need is a compiler (I use g++).

So let's say you want to print a vector of integers in C++. You could write your program in a file called printVector.cpp

It would look something like this:

    #include <vector>
    #include <iostream>

    using namespace std;

    void printVector(vector<int> vector) 
    {
        for (int element : vector)
        {
            cout << element << " ";
        }
        cout << endl;
    };

    int main
    {
        vector<int> intVector {1,2,3};
        printVector(intVector);
        return 0;
    } 

This is your source code. It doesn't do anything until compiled, so you'll have to go to the command line, get to the targer directory, and put in the appropriate command.

    g++ printVector.cpp -o "printVector"

This will put a binary into your directory called printVector. To run it, you type ./printVector

This will print 1 2 3 to the console.


### Getting Your Stuff More Organized ###

Okay so you can get one file to work. The problem is when you want to add more stuff. Then you'll have to organize things a bit more. This is why we have header files. In a header file, we define our functions that we want to use. Then we can write the implementation in the cpp file. 

So we might have a printVector.cpp and a printVector.hpp

In printVector.hpp, it's pretty simple. We could put
    #include <vector>
    #include <iostream>
    using namespace std;

    void printVector(vector<int> vector);

And then our printVector.cpp will just be
    #include "printVector.hpp"

    void printVector(vector<int> vector) 
    {
        for (int element : vector)
        {
            cout << element << " ";
        }
        cout << endl;
    };

    int main
    {
        vector<int> intVector {1,2,3};
        printVector(intVector);
        return 0;
    } 

We could compile this exactly the same way as before and it will work.


### Two cpp Files, Why Doesn't This Trash Programming Language Work ###

Let's change things around a little bit. Let's say we have our main program (this might be ROS). We don't want to pollute our ROS node files with a bunch of class definitions so we put those classes into their own cpp files. Then to further organize things, we have our header files. 

So now we have hpp file

    #include <vector>
    #include <iostream>
    using namespace std;

    void printVector(vector<int> vector);

Our cpp file

    void printVector(vector<int> vector) 
    {
        for (int element : vector)
        {
            cout << element << " ";
        }
        cout << endl;
    };

And our main file

    #include "printVector.hpp"

    vector<int> needToSendThisVectorOverRosNode {3,3,3};
    printVector(needToSendThisVectorOverRosNode);


Sick let's get this going. So we want to run main because obviously that's where our entry point is. Just gotta compile real quick!

    g++ main.cpp -o "cant_wait_to_use_my_program!"

ANNNNNNDDD you get this error

    main.cpp:(.text+0xac): undefined reference to `void printVector<int>(std::vector<int, std::allocator<int> >)


### WHAT THE FRICK I HATE THIS PROGRAMMING LANGUAGE ###

So what happened??

Well, you were optimisitic enough to think that your header file gives you everything you need to implement printVector. But it doesn't. And the reason why has to do with the #include statement. What is that even for anyway?

All this does is take whatever code is in the file you include, and plop it into the current file. That's literally it. 

So in main you have a program 

    #include "printVector.hpp"

    vector<int> needToSendThisVectorOverRosNode {3,3,3};
    printVector(needToSendThisVectorOverRosNode);

And all #include does is make the compiler do this pretty much 

    #include <vector>
    #include <iostream>
    using namespace std;

    void printVector(vector<int> vector);
    
    vector<int> needToSendThisVectorOverRosNode {3,3,3};
    printVector(needToSendThisVectorOverRosNode);

All it did was take the hpp file and put it into main. But then what do you notice?

What does printVector even mean????

The problem is now it seems like you never define printVector to the computer and it has no idea how to use it. 
Now you might be thinking oh cool I'll just include the cpp file!

HA! Even more wrong loser!

I'm kidding. Nah all you gotta do is compile your source code into a bunch of object files. Then you just have to link all those files, and you have your magically working program. That's it for this tutorial thanks for coming! 

Kidding again. Alright so how we do we get our object files. Easy, we can use the -c flag (this is nice too cuz you don't need a main for it to compile) 

    g++ main.cpp -c
    g++ printVector.cpp -c

Okay cool now we have main.o and printVector.o in our directory. That's promising. Let's link them into one file

    g++ -o "finalProgram" main.o printVector.o

This creates our binary finalProgram. And we run ./finalProgram and it will work!


Finally. But now the question is... what if you had 10 files to compile together? Does that mean you have to do g++ file1.cpp -c and g++ file2.cpp -c and file3.cpp ... etc every time you compile your program?

... well you could if you suck

### The Friend You Didn't Ask For - CMake ###

CMake is like when you're in middle school and have no friends, and some weird kid starts talking to you and somehow ends up becoming your friend simply for the fact that you have no other options (... wait is that just me)

CMake is a way for you to put all those cpp files into one place and compile them to make your life (theoretically) easier. Let's see how it works:

    cmake_minimum_required(VERSION 3.5)
    project(pid_controller)

    # Default to C++14
    if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
    endif()

    add_executable(pid_controller src/pid_controller.cpp
                                src/pid_params.cpp
                                src/scion_pid_controller.cpp)

    target_include_directories(pid_controller PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

    install(TARGETS
    pid_controller
    DESTINATION lib/${PROJECT_NAME})

This is a real CMakeList.txt file I made for the pid_controller. As you can see you new a few things. You should really keep all .cpp files in a src folder and all .hpp files in an include folder. Makes your life easier. The first half of the CMakeLists file is self explanatory. Just version, project name, then default to C++14 (before 11 is a bad idea). 

The add_executable is where to put your cpp files and whatever your executable will be named.

Then use target_include_directories to point to your include folder (#{CMAKE_CURRENT_SOURCE_DIR} is a variable that points to the current directory its in so you don't have to get the absolute path name)


To make this file useful you'll have to use a few commands, First:

    cmake CMakeLists.txt

This will start your build 

Then you'll have to make your executable 

    make ${whatever_you_put_as_your_executable_name}

And that will spit out your (digitial) liquid gold - your useable compiled file. And you''ll execute like normal ./my_executable


### Conclusion ###

Cool so that's about it. I'm not an expert so don't crucify me if there's something wrong here but I went through some pain figuring out to get all this to work so you don't have to. Hopefully it helps. It also really helps with organization if everyone codes the same way. Other good practices and for more detail you'll have to research yourself as well (for example, header guards).

My perspective is that we should have ROS all in one place, then classes in one place, and try to keep C++ files organized using cpp files in /src and hpp files in /include. Also always have a MakeFile so people can easily build your code. 

Alright I'll shut up now.
