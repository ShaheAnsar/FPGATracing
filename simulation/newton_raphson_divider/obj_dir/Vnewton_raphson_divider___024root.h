// Verilated -*- C++ -*-
// DESCRIPTION: Verilator output: Design internal header
// See Vnewton_raphson_divider.h for the primary calling header

#ifndef VERILATED_VNEWTON_RAPHSON_DIVIDER___024ROOT_H_
#define VERILATED_VNEWTON_RAPHSON_DIVIDER___024ROOT_H_  // guard

#include "verilated.h"


class Vnewton_raphson_divider__Syms;

class alignas(VL_CACHE_LINE_BYTES) Vnewton_raphson_divider___024root final : public VerilatedModule {
  public:

    // DESIGN SPECIFIC STATE
    VL_IN8(clk,0,0);
    VL_IN8(nRST,0,0);
    VL_IN8(request,0,0);
    VL_OUT8(ready,0,0);
    CData/*0:0*/ newton_raphson_divider__DOT__negative;
    CData/*4:0*/ newton_raphson_divider__DOT__shift_count;
    CData/*4:0*/ newton_raphson_divider__DOT__iter_count;
    CData/*1:0*/ newton_raphson_divider__DOT__shift_dir;
    CData/*0:0*/ newton_raphson_divider__DOT__fault;
    CData/*3:0*/ newton_raphson_divider__DOT__state;
    CData/*0:0*/ __Vtrigprevexpr___TOP__clk__0;
    CData/*0:0*/ __VactContinue;
    VL_IN(divider,31,0);
    VL_IN(dividend,31,0);
    VL_OUT(quotient,31,0);
    IData/*31:0*/ newton_raphson_divider__DOT__i_divider;
    IData/*31:0*/ newton_raphson_divider__DOT__i_dividend;
    IData/*31:0*/ __VactIterCount;
    QData/*62:0*/ newton_raphson_divider__DOT__i_quotient;
    VlUnpacked<CData/*0:0*/, 2> __Vm_traceActivity;
    VlTriggerVec<1> __VactTriggered;
    VlTriggerVec<1> __VnbaTriggered;

    // INTERNAL VARIABLES
    Vnewton_raphson_divider__Syms* const vlSymsp;

    // CONSTRUCTORS
    Vnewton_raphson_divider___024root(Vnewton_raphson_divider__Syms* symsp, const char* v__name);
    ~Vnewton_raphson_divider___024root();
    VL_UNCOPYABLE(Vnewton_raphson_divider___024root);

    // INTERNAL METHODS
    void __Vconfigure(bool first);
};


#endif  // guard
