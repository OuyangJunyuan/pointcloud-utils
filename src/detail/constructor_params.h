explicit PC_UTILS_CLASS (const Params &params) {
#define define(TYPE, NAME, VAL)  NAME = any_lexical_cast<TYPE>(params.at(#NAME));
    PC_UTILS_MEMBER_VARIABLE
#undef define
}