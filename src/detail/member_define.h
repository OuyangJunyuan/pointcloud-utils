//
// Created by ou on 2021/12/3.
//
private:
#define define(TYPE, NAME, VAL)           TYPE NAME VAL;
PC_UTILS_MEMBER_VARIABLE
#undef define

public:

#include "constructor_default.h"
#include "constructor_yaml.h"
#include "constructor_params.h"

std::string class_name() { return Type < PC_UTILS_CLASS < PointT >> ::str(); }

#undef PC_UTILS_MEMBER_VARIABLE
#undef PC_UTILS_CLASS