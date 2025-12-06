// Verilated -*- C++ -*-
// DESCRIPTION: Verilator output: Design implementation internals
// See Vnewton_raphson_divider.h for the primary calling header

#include "Vnewton_raphson_divider__pch.h"
#include "Vnewton_raphson_divider___024root.h"

VL_ATTR_COLD void Vnewton_raphson_divider___024root___eval_static(Vnewton_raphson_divider___024root* vlSelf) {
    VL_DEBUG_IF(VL_DBG_MSGF("+    Vnewton_raphson_divider___024root___eval_static\n"); );
    Vnewton_raphson_divider__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    auto& vlSelfRef = std::ref(*vlSelf).get();
    // Body
    vlSelfRef.__Vtrigprevexpr___TOP__clk__0 = vlSelfRef.clk;
}

VL_ATTR_COLD void Vnewton_raphson_divider___024root___eval_initial(Vnewton_raphson_divider___024root* vlSelf) {
    VL_DEBUG_IF(VL_DBG_MSGF("+    Vnewton_raphson_divider___024root___eval_initial\n"); );
    Vnewton_raphson_divider__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    auto& vlSelfRef = std::ref(*vlSelf).get();
}

VL_ATTR_COLD void Vnewton_raphson_divider___024root___eval_final(Vnewton_raphson_divider___024root* vlSelf) {
    VL_DEBUG_IF(VL_DBG_MSGF("+    Vnewton_raphson_divider___024root___eval_final\n"); );
    Vnewton_raphson_divider__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    auto& vlSelfRef = std::ref(*vlSelf).get();
}

VL_ATTR_COLD void Vnewton_raphson_divider___024root___eval_settle(Vnewton_raphson_divider___024root* vlSelf) {
    VL_DEBUG_IF(VL_DBG_MSGF("+    Vnewton_raphson_divider___024root___eval_settle\n"); );
    Vnewton_raphson_divider__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    auto& vlSelfRef = std::ref(*vlSelf).get();
}

#ifdef VL_DEBUG
VL_ATTR_COLD void Vnewton_raphson_divider___024root___dump_triggers__act(Vnewton_raphson_divider___024root* vlSelf) {
    VL_DEBUG_IF(VL_DBG_MSGF("+    Vnewton_raphson_divider___024root___dump_triggers__act\n"); );
    Vnewton_raphson_divider__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    auto& vlSelfRef = std::ref(*vlSelf).get();
    // Body
    if ((1U & (~ vlSelfRef.__VactTriggered.any()))) {
        VL_DBG_MSGF("         No triggers active\n");
    }
    if ((1ULL & vlSelfRef.__VactTriggered.word(0U))) {
        VL_DBG_MSGF("         'act' region trigger index 0 is active: @(posedge clk)\n");
    }
}
#endif  // VL_DEBUG

#ifdef VL_DEBUG
VL_ATTR_COLD void Vnewton_raphson_divider___024root___dump_triggers__nba(Vnewton_raphson_divider___024root* vlSelf) {
    VL_DEBUG_IF(VL_DBG_MSGF("+    Vnewton_raphson_divider___024root___dump_triggers__nba\n"); );
    Vnewton_raphson_divider__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    auto& vlSelfRef = std::ref(*vlSelf).get();
    // Body
    if ((1U & (~ vlSelfRef.__VnbaTriggered.any()))) {
        VL_DBG_MSGF("         No triggers active\n");
    }
    if ((1ULL & vlSelfRef.__VnbaTriggered.word(0U))) {
        VL_DBG_MSGF("         'nba' region trigger index 0 is active: @(posedge clk)\n");
    }
}
#endif  // VL_DEBUG

VL_ATTR_COLD void Vnewton_raphson_divider___024root___ctor_var_reset(Vnewton_raphson_divider___024root* vlSelf) {
    VL_DEBUG_IF(VL_DBG_MSGF("+    Vnewton_raphson_divider___024root___ctor_var_reset\n"); );
    Vnewton_raphson_divider__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    auto& vlSelfRef = std::ref(*vlSelf).get();
    // Body
    const uint64_t __VscopeHash = VL_MURMUR64_HASH(vlSelf->name());
    vlSelf->clk = VL_SCOPED_RAND_RESET_I(1, __VscopeHash, 16707436170211756652ull);
    vlSelf->nRST = VL_SCOPED_RAND_RESET_I(1, __VscopeHash, 9596079045119723318ull);
    vlSelf->request = VL_SCOPED_RAND_RESET_I(1, __VscopeHash, 16104405464408049176ull);
    vlSelf->divider = VL_SCOPED_RAND_RESET_I(32, __VscopeHash, 9613828308581145985ull);
    vlSelf->dividend = VL_SCOPED_RAND_RESET_I(32, __VscopeHash, 9925771335595998127ull);
    vlSelf->quotient = VL_SCOPED_RAND_RESET_I(32, __VscopeHash, 11798559827265539290ull);
    vlSelf->ready = VL_SCOPED_RAND_RESET_I(1, __VscopeHash, 898948264233693212ull);
    vlSelf->newton_raphson_divider__DOT__i_divider = VL_SCOPED_RAND_RESET_I(32, __VscopeHash, 12383072251154037945ull);
    vlSelf->newton_raphson_divider__DOT__i_dividend = VL_SCOPED_RAND_RESET_I(32, __VscopeHash, 4561427658418576733ull);
    vlSelf->newton_raphson_divider__DOT__i_quotient = VL_SCOPED_RAND_RESET_Q(63, __VscopeHash, 4013847809054318484ull);
    vlSelf->newton_raphson_divider__DOT__negative = VL_SCOPED_RAND_RESET_I(1, __VscopeHash, 2057326432424549333ull);
    vlSelf->newton_raphson_divider__DOT__shift_count = VL_SCOPED_RAND_RESET_I(5, __VscopeHash, 1932836368693672449ull);
    vlSelf->newton_raphson_divider__DOT__iter_count = VL_SCOPED_RAND_RESET_I(5, __VscopeHash, 12990734485184937490ull);
    vlSelf->newton_raphson_divider__DOT__shift_dir = VL_SCOPED_RAND_RESET_I(2, __VscopeHash, 2756917204498758384ull);
    vlSelf->newton_raphson_divider__DOT__fault = VL_SCOPED_RAND_RESET_I(1, __VscopeHash, 10334746624859638622ull);
    vlSelf->newton_raphson_divider__DOT__state = VL_SCOPED_RAND_RESET_I(4, __VscopeHash, 11291272151455092292ull);
    vlSelf->__Vtrigprevexpr___TOP__clk__0 = VL_SCOPED_RAND_RESET_I(1, __VscopeHash, 9526919608049418986ull);
    for (int __Vi0 = 0; __Vi0 < 2; ++__Vi0) {
        vlSelf->__Vm_traceActivity[__Vi0] = 0;
    }
}
