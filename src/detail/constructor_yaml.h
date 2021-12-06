explicit PC_UTILS_CLASS (const YAML::Node &params) {
#define define(TYPE, NAME, VAL) NAME = params[#NAME].as<TYPE>();
    PC_UTILS_MEMBER_VARIABLE
#undef define
}