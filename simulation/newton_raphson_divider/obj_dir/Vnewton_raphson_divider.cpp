// Verilated -*- C++ -*-
// DESCRIPTION: Verilator output: Model implementation (design independent parts)

#include "Vnewton_raphson_divider__pch.h"
#include "verilated_vcd_c.h"

//============================================================
// Constructors

Vnewton_raphson_divider::Vnewton_raphson_divider(VerilatedContext* _vcontextp__, const char* _vcname__)
    : VerilatedModel{*_vcontextp__}
    , vlSymsp{new Vnewton_raphson_divider__Syms(contextp(), _vcname__, this)}
    , clk{vlSymsp->TOP.clk}
    , nRST{vlSymsp->TOP.nRST}
    , request{vlSymsp->TOP.request}
    , ready{vlSymsp->TOP.ready}
    , divider{vlSymsp->TOP.divider}
    , dividend{vlSymsp->TOP.dividend}
    , quotient{vlSymsp->TOP.quotient}
    , rootp{&(vlSymsp->TOP)}
{
    // Register model with the context
    contextp()->addModel(this);
    contextp()->traceBaseModelCbAdd(
        [this](VerilatedTraceBaseC* tfp, int levels, int options) { traceBaseModel(tfp, levels, options); });
}

Vnewton_raphson_divider::Vnewton_raphson_divider(const char* _vcname__)
    : Vnewton_raphson_divider(Verilated::threadContextp(), _vcname__)
{
}

//============================================================
// Destructor

Vnewton_raphson_divider::~Vnewton_raphson_divider() {
    delete vlSymsp;
}

//============================================================
// Evaluation function

#ifdef VL_DEBUG
void Vnewton_raphson_divider___024root___eval_debug_assertions(Vnewton_raphson_divider___024root* vlSelf);
#endif  // VL_DEBUG
void Vnewton_raphson_divider___024root___eval_static(Vnewton_raphson_divider___024root* vlSelf);
void Vnewton_raphson_divider___024root___eval_initial(Vnewton_raphson_divider___024root* vlSelf);
void Vnewton_raphson_divider___024root___eval_settle(Vnewton_raphson_divider___024root* vlSelf);
void Vnewton_raphson_divider___024root___eval(Vnewton_raphson_divider___024root* vlSelf);

void Vnewton_raphson_divider::eval_step() {
    VL_DEBUG_IF(VL_DBG_MSGF("+++++TOP Evaluate Vnewton_raphson_divider::eval_step\n"); );
#ifdef VL_DEBUG
    // Debug assertions
    Vnewton_raphson_divider___024root___eval_debug_assertions(&(vlSymsp->TOP));
#endif  // VL_DEBUG
    vlSymsp->__Vm_activity = true;
    vlSymsp->__Vm_deleter.deleteAll();
    if (VL_UNLIKELY(!vlSymsp->__Vm_didInit)) {
        vlSymsp->__Vm_didInit = true;
        VL_DEBUG_IF(VL_DBG_MSGF("+ Initial\n"););
        Vnewton_raphson_divider___024root___eval_static(&(vlSymsp->TOP));
        Vnewton_raphson_divider___024root___eval_initial(&(vlSymsp->TOP));
        Vnewton_raphson_divider___024root___eval_settle(&(vlSymsp->TOP));
    }
    VL_DEBUG_IF(VL_DBG_MSGF("+ Eval\n"););
    Vnewton_raphson_divider___024root___eval(&(vlSymsp->TOP));
    // Evaluate cleanup
    Verilated::endOfEval(vlSymsp->__Vm_evalMsgQp);
}

//============================================================
// Events and timing
bool Vnewton_raphson_divider::eventsPending() { return false; }

uint64_t Vnewton_raphson_divider::nextTimeSlot() {
    VL_FATAL_MT(__FILE__, __LINE__, "", "No delays in the design");
    return 0;
}

//============================================================
// Utilities

const char* Vnewton_raphson_divider::name() const {
    return vlSymsp->name();
}

//============================================================
// Invoke final blocks

void Vnewton_raphson_divider___024root___eval_final(Vnewton_raphson_divider___024root* vlSelf);

VL_ATTR_COLD void Vnewton_raphson_divider::final() {
    Vnewton_raphson_divider___024root___eval_final(&(vlSymsp->TOP));
}

//============================================================
// Implementations of abstract methods from VerilatedModel

const char* Vnewton_raphson_divider::hierName() const { return vlSymsp->name(); }
const char* Vnewton_raphson_divider::modelName() const { return "Vnewton_raphson_divider"; }
unsigned Vnewton_raphson_divider::threads() const { return 1; }
void Vnewton_raphson_divider::prepareClone() const { contextp()->prepareClone(); }
void Vnewton_raphson_divider::atClone() const {
    contextp()->threadPoolpOnClone();
}
std::unique_ptr<VerilatedTraceConfig> Vnewton_raphson_divider::traceConfig() const {
    return std::unique_ptr<VerilatedTraceConfig>{new VerilatedTraceConfig{false, false, false}};
};

//============================================================
// Trace configuration

void Vnewton_raphson_divider___024root__trace_decl_types(VerilatedVcd* tracep);

void Vnewton_raphson_divider___024root__trace_init_top(Vnewton_raphson_divider___024root* vlSelf, VerilatedVcd* tracep);

VL_ATTR_COLD static void trace_init(void* voidSelf, VerilatedVcd* tracep, uint32_t code) {
    // Callback from tracep->open()
    Vnewton_raphson_divider___024root* const __restrict vlSelf VL_ATTR_UNUSED = static_cast<Vnewton_raphson_divider___024root*>(voidSelf);
    Vnewton_raphson_divider__Syms* const __restrict vlSymsp VL_ATTR_UNUSED = vlSelf->vlSymsp;
    if (!vlSymsp->_vm_contextp__->calcUnusedSigs()) {
        VL_FATAL_MT(__FILE__, __LINE__, __FILE__,
            "Turning on wave traces requires Verilated::traceEverOn(true) call before time 0.");
    }
    vlSymsp->__Vm_baseCode = code;
    tracep->pushPrefix(std::string{vlSymsp->name()}, VerilatedTracePrefixType::SCOPE_MODULE);
    Vnewton_raphson_divider___024root__trace_decl_types(tracep);
    Vnewton_raphson_divider___024root__trace_init_top(vlSelf, tracep);
    tracep->popPrefix();
}

VL_ATTR_COLD void Vnewton_raphson_divider___024root__trace_register(Vnewton_raphson_divider___024root* vlSelf, VerilatedVcd* tracep);

VL_ATTR_COLD void Vnewton_raphson_divider::traceBaseModel(VerilatedTraceBaseC* tfp, int levels, int options) {
    (void)levels; (void)options;
    VerilatedVcdC* const stfp = dynamic_cast<VerilatedVcdC*>(tfp);
    if (VL_UNLIKELY(!stfp)) {
        vl_fatal(__FILE__, __LINE__, __FILE__,"'Vnewton_raphson_divider::trace()' called on non-VerilatedVcdC object;"
            " use --trace-fst with VerilatedFst object, and --trace-vcd with VerilatedVcd object");
    }
    stfp->spTrace()->addModel(this);
    stfp->spTrace()->addInitCb(&trace_init, &(vlSymsp->TOP));
    Vnewton_raphson_divider___024root__trace_register(&(vlSymsp->TOP), stfp->spTrace());
}
