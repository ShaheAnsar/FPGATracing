// Verilated -*- C++ -*-
// DESCRIPTION: Verilator output: Design implementation internals
// See Vnewton_raphson_divider.h for the primary calling header

#include "Vnewton_raphson_divider__pch.h"
#include "Vnewton_raphson_divider___024root.h"

void Vnewton_raphson_divider___024root___eval_act(Vnewton_raphson_divider___024root* vlSelf) {
    VL_DEBUG_IF(VL_DBG_MSGF("+    Vnewton_raphson_divider___024root___eval_act\n"); );
    Vnewton_raphson_divider__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    auto& vlSelfRef = std::ref(*vlSelf).get();
}

void Vnewton_raphson_divider___024root___nba_sequent__TOP__0(Vnewton_raphson_divider___024root* vlSelf);

void Vnewton_raphson_divider___024root___eval_nba(Vnewton_raphson_divider___024root* vlSelf) {
    VL_DEBUG_IF(VL_DBG_MSGF("+    Vnewton_raphson_divider___024root___eval_nba\n"); );
    Vnewton_raphson_divider__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    auto& vlSelfRef = std::ref(*vlSelf).get();
    // Body
    if ((1ULL & vlSelfRef.__VnbaTriggered.word(0U))) {
        Vnewton_raphson_divider___024root___nba_sequent__TOP__0(vlSelf);
        vlSelfRef.__Vm_traceActivity[1U] = 1U;
    }
}

VL_INLINE_OPT void Vnewton_raphson_divider___024root___nba_sequent__TOP__0(Vnewton_raphson_divider___024root* vlSelf) {
    VL_DEBUG_IF(VL_DBG_MSGF("+    Vnewton_raphson_divider___024root___nba_sequent__TOP__0\n"); );
    Vnewton_raphson_divider__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    auto& vlSelfRef = std::ref(*vlSelf).get();
    // Init
    CData/*3:0*/ __Vdly__newton_raphson_divider__DOT__state;
    __Vdly__newton_raphson_divider__DOT__state = 0;
    CData/*4:0*/ __Vdly__newton_raphson_divider__DOT__shift_count;
    __Vdly__newton_raphson_divider__DOT__shift_count = 0;
    IData/*31:0*/ __Vdly__newton_raphson_divider__DOT__i_divider;
    __Vdly__newton_raphson_divider__DOT__i_divider = 0;
    IData/*31:0*/ __VdlyMask__newton_raphson_divider__DOT__i_divider;
    __VdlyMask__newton_raphson_divider__DOT__i_divider = 0;
    // Body
    __Vdly__newton_raphson_divider__DOT__state = vlSelfRef.newton_raphson_divider__DOT__state;
    __Vdly__newton_raphson_divider__DOT__shift_count 
        = vlSelfRef.newton_raphson_divider__DOT__shift_count;
    if (vlSelfRef.nRST) {
        if ((8U & (IData)(vlSelfRef.newton_raphson_divider__DOT__state))) {
            vlSelfRef.newton_raphson_divider__DOT__fault = 1U;
        } else if ((4U & (IData)(vlSelfRef.newton_raphson_divider__DOT__state))) {
            vlSelfRef.newton_raphson_divider__DOT__fault = 1U;
        } else if ((2U & (IData)(vlSelfRef.newton_raphson_divider__DOT__state))) {
            if ((1U & (IData)(vlSelfRef.newton_raphson_divider__DOT__state))) {
                vlSelfRef.ready = 1U;
                vlSelfRef.quotient = (IData)((0x7fffffffffffffffULL 
                                              & ((0U 
                                                  == (IData)(vlSelfRef.newton_raphson_divider__DOT__shift_dir))
                                                  ? 
                                                 (VL_SHIFTR_QQI(63,63,32, 
                                                                (0x7fffffffffffffffULL 
                                                                 & (vlSelfRef.newton_raphson_divider__DOT__i_quotient 
                                                                    * (QData)((IData)(vlSelfRef.newton_raphson_divider__DOT__i_dividend)))), 0x10U) 
                                                  << (IData)(vlSelfRef.newton_raphson_divider__DOT__shift_count))
                                                  : 
                                                 ((0x7fffffffffffffffULL 
                                                   & VL_SHIFTR_QQI(63,63,32, 
                                                                   (0x7fffffffffffffffULL 
                                                                    & (vlSelfRef.newton_raphson_divider__DOT__i_quotient 
                                                                       * (QData)((IData)(vlSelfRef.newton_raphson_divider__DOT__i_dividend)))), 0x10U)) 
                                                  >> (IData)(vlSelfRef.newton_raphson_divider__DOT__shift_count)))));
                __Vdly__newton_raphson_divider__DOT__state = 0U;
            } else if ((4U > (IData)(vlSelfRef.newton_raphson_divider__DOT__iter_count))) {
                vlSelfRef.newton_raphson_divider__DOT__i_quotient 
                    = (0x7fffffffffffffffULL & VL_SHIFTR_QQI(63,63,32, 
                                                             (0x7fffffffffffffffULL 
                                                              & (vlSelfRef.newton_raphson_divider__DOT__i_quotient 
                                                                 * 
                                                                 (0x7fffffffffffffffULL 
                                                                  & (0x20000ULL 
                                                                     - 
                                                                     VL_SHIFTR_QQI(63,63,32, 
                                                                                (0x7fffffffffffffffULL 
                                                                                & (vlSelfRef.newton_raphson_divider__DOT__i_quotient 
                                                                                * (QData)((IData)(vlSelfRef.newton_raphson_divider__DOT__i_divider)))), 0x10U))))), 0x10U));
                vlSelfRef.newton_raphson_divider__DOT__iter_count 
                    = (0x1fU & ((IData)(1U) + (IData)(vlSelfRef.newton_raphson_divider__DOT__iter_count)));
            } else {
                __Vdly__newton_raphson_divider__DOT__state = 3U;
            }
        } else if ((1U & (IData)(vlSelfRef.newton_raphson_divider__DOT__state))) {
            if ((0x40000U < vlSelfRef.newton_raphson_divider__DOT__i_divider)) {
                vlSelfRef.newton_raphson_divider__DOT__i_divider 
                    = VL_SHIFTR_III(32,32,32, vlSelfRef.newton_raphson_divider__DOT__i_divider, 3U);
                __Vdly__newton_raphson_divider__DOT__shift_count 
                    = (0x1fU & ((IData)(3U) + (IData)(vlSelfRef.newton_raphson_divider__DOT__shift_count)));
                vlSelfRef.newton_raphson_divider__DOT__shift_dir = 1U;
            } else if ((0x20000U < vlSelfRef.newton_raphson_divider__DOT__i_divider)) {
                vlSelfRef.newton_raphson_divider__DOT__i_divider 
                    = VL_SHIFTR_III(32,32,32, vlSelfRef.newton_raphson_divider__DOT__i_divider, 2U);
                __Vdly__newton_raphson_divider__DOT__shift_count 
                    = (0x1fU & ((IData)(2U) + (IData)(vlSelfRef.newton_raphson_divider__DOT__shift_count)));
                vlSelfRef.newton_raphson_divider__DOT__shift_dir = 1U;
            } else if ((0x10000U < vlSelfRef.newton_raphson_divider__DOT__i_divider)) {
                vlSelfRef.newton_raphson_divider__DOT__i_divider 
                    = VL_SHIFTR_III(32,32,32, vlSelfRef.newton_raphson_divider__DOT__i_divider, 1U);
                __Vdly__newton_raphson_divider__DOT__shift_count 
                    = (0x1fU & ((IData)(1U) + (IData)(vlSelfRef.newton_raphson_divider__DOT__shift_count)));
                vlSelfRef.newton_raphson_divider__DOT__shift_dir = 1U;
            } else if ((0x2000U > vlSelfRef.newton_raphson_divider__DOT__i_divider)) {
                vlSelfRef.newton_raphson_divider__DOT__i_divider 
                    = VL_SHIFTL_III(32,32,32, vlSelfRef.newton_raphson_divider__DOT__i_divider, 3U);
                __Vdly__newton_raphson_divider__DOT__shift_count 
                    = (0x1fU & ((IData)(3U) + (IData)(vlSelfRef.newton_raphson_divider__DOT__shift_count)));
                vlSelfRef.newton_raphson_divider__DOT__shift_dir = 0U;
            } else if ((0x4000U > vlSelfRef.newton_raphson_divider__DOT__i_divider)) {
                vlSelfRef.newton_raphson_divider__DOT__i_divider 
                    = VL_SHIFTL_III(32,32,32, vlSelfRef.newton_raphson_divider__DOT__i_divider, 2U);
                __Vdly__newton_raphson_divider__DOT__shift_count 
                    = (0x1fU & ((IData)(2U) + (IData)(vlSelfRef.newton_raphson_divider__DOT__shift_count)));
                vlSelfRef.newton_raphson_divider__DOT__shift_dir = 0U;
            } else if ((0x8000U > vlSelfRef.newton_raphson_divider__DOT__i_divider)) {
                vlSelfRef.newton_raphson_divider__DOT__i_divider 
                    = VL_SHIFTL_III(32,32,32, vlSelfRef.newton_raphson_divider__DOT__i_divider, 1U);
                __Vdly__newton_raphson_divider__DOT__shift_count 
                    = (0x1fU & ((IData)(1U) + (IData)(vlSelfRef.newton_raphson_divider__DOT__shift_count)));
                vlSelfRef.newton_raphson_divider__DOT__shift_dir = 0U;
            } else {
                vlSelfRef.newton_raphson_divider__DOT__i_quotient = 0x1c000ULL;
                __Vdly__newton_raphson_divider__DOT__state = 2U;
            }
        } else {
            __Vdly__newton_raphson_divider__DOT__i_divider 
                = vlSelfRef.divider;
            __VdlyMask__newton_raphson_divider__DOT__i_divider = 0xffffffffU;
            vlSelfRef.newton_raphson_divider__DOT__i_dividend 
                = vlSelfRef.dividend;
            if (vlSelfRef.request) {
                vlSelfRef.ready = 0U;
                __Vdly__newton_raphson_divider__DOT__state = 1U;
            }
        }
    } else {
        vlSelfRef.newton_raphson_divider__DOT__i_quotient = 0ULL;
        __Vdly__newton_raphson_divider__DOT__shift_count = 0U;
        vlSelfRef.newton_raphson_divider__DOT__fault = 0U;
        vlSelfRef.newton_raphson_divider__DOT__i_dividend = 0U;
        __Vdly__newton_raphson_divider__DOT__i_divider = 0U;
        __VdlyMask__newton_raphson_divider__DOT__i_divider = 0xffffffffU;
        vlSelfRef.newton_raphson_divider__DOT__shift_dir = 0U;
        __Vdly__newton_raphson_divider__DOT__state = 0U;
    }
    vlSelfRef.newton_raphson_divider__DOT__state = __Vdly__newton_raphson_divider__DOT__state;
    vlSelfRef.newton_raphson_divider__DOT__shift_count 
        = __Vdly__newton_raphson_divider__DOT__shift_count;
    vlSelfRef.newton_raphson_divider__DOT__i_divider 
        = ((__Vdly__newton_raphson_divider__DOT__i_divider 
            & __VdlyMask__newton_raphson_divider__DOT__i_divider) 
           | (vlSelfRef.newton_raphson_divider__DOT__i_divider 
              & (~ __VdlyMask__newton_raphson_divider__DOT__i_divider)));
    __VdlyMask__newton_raphson_divider__DOT__i_divider = 0U;
}

void Vnewton_raphson_divider___024root___eval_triggers__act(Vnewton_raphson_divider___024root* vlSelf);

bool Vnewton_raphson_divider___024root___eval_phase__act(Vnewton_raphson_divider___024root* vlSelf) {
    VL_DEBUG_IF(VL_DBG_MSGF("+    Vnewton_raphson_divider___024root___eval_phase__act\n"); );
    Vnewton_raphson_divider__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    auto& vlSelfRef = std::ref(*vlSelf).get();
    // Init
    VlTriggerVec<1> __VpreTriggered;
    CData/*0:0*/ __VactExecute;
    // Body
    Vnewton_raphson_divider___024root___eval_triggers__act(vlSelf);
    __VactExecute = vlSelfRef.__VactTriggered.any();
    if (__VactExecute) {
        __VpreTriggered.andNot(vlSelfRef.__VactTriggered, vlSelfRef.__VnbaTriggered);
        vlSelfRef.__VnbaTriggered.thisOr(vlSelfRef.__VactTriggered);
        Vnewton_raphson_divider___024root___eval_act(vlSelf);
    }
    return (__VactExecute);
}

bool Vnewton_raphson_divider___024root___eval_phase__nba(Vnewton_raphson_divider___024root* vlSelf) {
    VL_DEBUG_IF(VL_DBG_MSGF("+    Vnewton_raphson_divider___024root___eval_phase__nba\n"); );
    Vnewton_raphson_divider__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    auto& vlSelfRef = std::ref(*vlSelf).get();
    // Init
    CData/*0:0*/ __VnbaExecute;
    // Body
    __VnbaExecute = vlSelfRef.__VnbaTriggered.any();
    if (__VnbaExecute) {
        Vnewton_raphson_divider___024root___eval_nba(vlSelf);
        vlSelfRef.__VnbaTriggered.clear();
    }
    return (__VnbaExecute);
}

#ifdef VL_DEBUG
VL_ATTR_COLD void Vnewton_raphson_divider___024root___dump_triggers__nba(Vnewton_raphson_divider___024root* vlSelf);
#endif  // VL_DEBUG
#ifdef VL_DEBUG
VL_ATTR_COLD void Vnewton_raphson_divider___024root___dump_triggers__act(Vnewton_raphson_divider___024root* vlSelf);
#endif  // VL_DEBUG

void Vnewton_raphson_divider___024root___eval(Vnewton_raphson_divider___024root* vlSelf) {
    VL_DEBUG_IF(VL_DBG_MSGF("+    Vnewton_raphson_divider___024root___eval\n"); );
    Vnewton_raphson_divider__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    auto& vlSelfRef = std::ref(*vlSelf).get();
    // Init
    IData/*31:0*/ __VnbaIterCount;
    CData/*0:0*/ __VnbaContinue;
    // Body
    __VnbaIterCount = 0U;
    __VnbaContinue = 1U;
    while (__VnbaContinue) {
        if (VL_UNLIKELY(((0x64U < __VnbaIterCount)))) {
#ifdef VL_DEBUG
            Vnewton_raphson_divider___024root___dump_triggers__nba(vlSelf);
#endif
            VL_FATAL_MT("../../picorv_learn.v", 1, "", "NBA region did not converge.");
        }
        __VnbaIterCount = ((IData)(1U) + __VnbaIterCount);
        __VnbaContinue = 0U;
        vlSelfRef.__VactIterCount = 0U;
        vlSelfRef.__VactContinue = 1U;
        while (vlSelfRef.__VactContinue) {
            if (VL_UNLIKELY(((0x64U < vlSelfRef.__VactIterCount)))) {
#ifdef VL_DEBUG
                Vnewton_raphson_divider___024root___dump_triggers__act(vlSelf);
#endif
                VL_FATAL_MT("../../picorv_learn.v", 1, "", "Active region did not converge.");
            }
            vlSelfRef.__VactIterCount = ((IData)(1U) 
                                         + vlSelfRef.__VactIterCount);
            vlSelfRef.__VactContinue = 0U;
            if (Vnewton_raphson_divider___024root___eval_phase__act(vlSelf)) {
                vlSelfRef.__VactContinue = 1U;
            }
        }
        if (Vnewton_raphson_divider___024root___eval_phase__nba(vlSelf)) {
            __VnbaContinue = 1U;
        }
    }
}

#ifdef VL_DEBUG
void Vnewton_raphson_divider___024root___eval_debug_assertions(Vnewton_raphson_divider___024root* vlSelf) {
    VL_DEBUG_IF(VL_DBG_MSGF("+    Vnewton_raphson_divider___024root___eval_debug_assertions\n"); );
    Vnewton_raphson_divider__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    auto& vlSelfRef = std::ref(*vlSelf).get();
    // Body
    if (VL_UNLIKELY(((vlSelfRef.clk & 0xfeU)))) {
        Verilated::overWidthError("clk");}
    if (VL_UNLIKELY(((vlSelfRef.nRST & 0xfeU)))) {
        Verilated::overWidthError("nRST");}
    if (VL_UNLIKELY(((vlSelfRef.request & 0xfeU)))) {
        Verilated::overWidthError("request");}
}
#endif  // VL_DEBUG
