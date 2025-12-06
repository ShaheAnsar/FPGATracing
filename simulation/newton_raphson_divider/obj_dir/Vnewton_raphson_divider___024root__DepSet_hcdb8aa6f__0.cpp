// Verilated -*- C++ -*-
// DESCRIPTION: Verilator output: Design implementation internals
// See Vnewton_raphson_divider.h for the primary calling header

#include "Vnewton_raphson_divider__pch.h"
#include "Vnewton_raphson_divider__Syms.h"
#include "Vnewton_raphson_divider___024root.h"

#ifdef VL_DEBUG
VL_ATTR_COLD void Vnewton_raphson_divider___024root___dump_triggers__act(Vnewton_raphson_divider___024root* vlSelf);
#endif  // VL_DEBUG

void Vnewton_raphson_divider___024root___eval_triggers__act(Vnewton_raphson_divider___024root* vlSelf) {
    VL_DEBUG_IF(VL_DBG_MSGF("+    Vnewton_raphson_divider___024root___eval_triggers__act\n"); );
    Vnewton_raphson_divider__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    auto& vlSelfRef = std::ref(*vlSelf).get();
    // Body
    vlSelfRef.__VactTriggered.setBit(0U, ((IData)(vlSelfRef.clk) 
                                          & (~ (IData)(vlSelfRef.__Vtrigprevexpr___TOP__clk__0))));
    vlSelfRef.__Vtrigprevexpr___TOP__clk__0 = vlSelfRef.clk;
#ifdef VL_DEBUG
    if (VL_UNLIKELY(vlSymsp->_vm_contextp__->debug())) {
        Vnewton_raphson_divider___024root___dump_triggers__act(vlSelf);
    }
#endif
}
