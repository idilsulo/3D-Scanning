#pragma once
#include <iostream>
#include <sstream>

struct Student
{
    std::string name;
    int passed_exercises;
    int borderline_exercises;

    std::string info()
    {
        std::stringstream info;
        info << "Student " << name;
        info << " has " << passed_exercises;
        info << " passed and " << borderline_exercises << " borderline exercises.";

        return info.str();
    }

    bool gets_bonus() 
    {
        if (passed_exercises >= 4 && borderline_exercises <= 1)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
};


