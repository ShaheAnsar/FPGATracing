// Verilated -*- C++ -*-
// DESCRIPTION: Verilator output: Symbol table implementation internals

#include "Vnewton_raphson_divider__pch.h"
#include "Vnewton_raphson_divider.h"
#include "Vnewton_raphson_divider___024root.h"

// FUNCTIONS
Vnewton_raphson_divider__Syms::~Vnewton_raphson_divider__Syms()
{
}

Vnewton_raphson_divider__Syms::Vnewton_raphson_divider__Syms(VerilatedContext* contextp, const char* namep, Vnewton_raphson_divider* modelp)
    : VerilatedSyms{contextp}
    // Setup internal state of the Syms class
    , __Vm_modelp{modelp}
    // Setup module instances
    , TOP{this, namep}
{
        // Check resources
        Verilated::stackCheck(31);
    // Configure time unit / time precision
    _vm_contextp__->timeunit(-9);
    _vm_contextp__->timeprecision(-12);
    // Setup each module's pointers to their submodules
    // Setup each module's pointer back to symbol table (for public functions)
    TOP.__Vconfigure(true);
    // Setup export functions
    for (int __Vfinal = 0; __Vfinal < 2; ++__Vfinal) {
    }
}
