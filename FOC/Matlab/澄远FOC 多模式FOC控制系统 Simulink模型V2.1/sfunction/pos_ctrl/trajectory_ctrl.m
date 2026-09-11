defs = [];

% sfun_PosCtrlInit
def = legacy_code('initialize');
def.SFunctionName = 'sfun_PosCtrlInit';
def.OutputFcnSpec = 'void PosCtrlInit(PosCtrlHandle_t y1[1])';
def.HeaderFiles   = {'trajectory_ctrl.h'};
def.SourceFiles   = {'trajectory_ctrl.c'};
defs = [defs; def];

% sfun_PosCtrPlan
def = legacy_code('initialize');
def.SFunctionName = 'sfun_PosCtrPlan';
def.OutputFcnSpec = 'uint8 y1 = PosCtrPlan(single u1, single u2, single u3, PosCtrlHandle_t u4[1], PosCtrlHandle_t y2[1])';
def.HeaderFiles   = {'trajectory_ctrl.h'};
def.SourceFiles   = {'trajectory_ctrl.c'};
defs = [defs; def];

% sfun_PosCtrPlanExec
def = legacy_code('initialize');
def.SFunctionName = 'sfun_PosCtrPlanExec';
def.OutputFcnSpec = 'uint8 y1 = PosCtrPlanExec(PosCtrlHandle_t u1[1], PosCtrlHandle_t y2[1])';
def.HeaderFiles   = {'trajectory_ctrl.h'};
def.SourceFiles   = {'trajectory_ctrl.c'};
defs = [defs; def];

legacy_code('sfcn_cmex_generate', defs);
legacy_code('compile', defs);
legacy_code('slblock_generate', defs, 'trajectory_ctrl_model');
legacy_code('sfcn_tlc_generate', defs);
