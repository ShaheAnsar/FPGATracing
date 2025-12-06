// Verilated -*- C++ -*-
// DESCRIPTION: Verilator output: Tracing implementation internals
#include "verilated_vcd_c.h"
#include "Vnewton_raphson_divider__Syms.h"


void Vnewton_raphson_divider___024root__trace_chg_0_sub_0(Vnewton_raphson_divider___024root* vlSelf, VerilatedVcd::Buffer* bufp);

void Vnewton_raphson_divider___024root__trace_chg_0(void* voidSelf, VerilatedVcd::Buffer* bufp) {
    VL_DEBUG_IF(VL_DBG_MSGF("+    Vnewton_raphson_divider___024root__trace_chg_0\n"); );
    // Init
    Vnewton_raphson_divider___024root* const __restrict vlSelf VL_ATTR_UNUSED = static_cast<Vnewton_raphson_divider___024root*>(voidSelf);
    Vnewton_raphson_divider__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    if (VL_UNLIKELY(!vlSymsp->__Vm_activity)) return;
    // Body
    Vnewton_raphson_divider___024root__trace_chg_0_sub_0((&vlSymsp->TOP), bufp);
}

void Vnewton_raphson_divider___024root__trace_chg_0_sub_0(Vnewton_raphson_divider___024root* vlSelf, VerilatedVcd::Buffer* bufp) {
    VL_DEBUG_IF(VL_DBG_MSGF("+    Vnewton_raphson_divider___024root__trace_chg_0_sub_0\n"); );
    Vnewton_raphson_divider__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    auto& vlSelfRef = std::ref(*vlSelf).get();
    // Init
    uint32_t* const oldp VL_ATTR_UNUSED = bufp->oldp(vlSymsp->__Vm_baseCode + 1);
    // Body
    if (VL_UNLIKELY((vlSelfRef.__Vm_traceActivity[1U]))) {
        bufp->chgIData(oldp+0,(vlSelfRef.newton_raphson_divider__DOT__i_divider),32);
        bufp->chgIData(oldp+1,(vlSelfRef.newton_raphson_divider__DOT__i_dividend),32);
        bufp->chgQData(oldp+2,(vlSelfRef.newton_raphson_divider__DOT__i_quotient),63);
        bufp->chgCData(oldp+4,(vlSelfRef.newton_raphson_divider__DOT__shift_count),5);
        bufp->chgCData(oldp+5,(vlSelfRef.newton_raphson_divider__DOT__iter_count),5);
        bufp->chgCData(oldp+6,(vlSelfRef.newton_raphson_divider__DOT__shift_dir),2);
        bufp->chgBit(oldp+7,(vlSelfRef.newton_raphson_divider__DOT__fault));
        bufp->chgCData(oldp+8,(vlSelfRef.newton_raphson_divider__DOT__state),4);
    }
    bufp->chgBit(oldp+9,(vlSelfRef.clk));
    bufp->chgBit(oldp+10,(vlSelfRef.nRST));
    bufp->chgBit(oldp+11,(vlSelfRef.request));
    bufp->chgIData(oldp+12,(vlSelfRef.divider),32);
    bufp->chgIData(oldp+13,(vlSelfRef.dividend),32);
    bufp->chgIData(oldp+14,(vlSelfRef.quotient),32);
    bufp->chgBit(oldp+15,(vlSelfRef.ready));
}

void Vnewton_raphson_divider___024root__trace_cleanup(void* voidSelf, VerilatedVcd* /*unused*/) {
    VL_DEBUG_IF(VL_DBG_MSGF("+    Vnewton_raphson_divider___024root__trace_cleanup\n"); );
    // Init
    Vnewton_raphson_divider___024root* const __restrict vlSelf VL_ATTR_UNUSED = static_cast<Vnewton_raphson_divider___024root*>(voidSelf);
    Vnewton_raphson_divider__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    // Body
    vlSymsp->__Vm_activity = false;
    vlSymsp->TOP.__Vm_traceActivity[0U] = 0U;
    vlSymsp->TOP.__Vm_traceActivity[1U] = 0U;
}
