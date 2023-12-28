#pragma once
#ifndef NDEBUG
    #define ASSERT(condition) assert(condition)
#else
    #define ASSERT(condition)
#endif