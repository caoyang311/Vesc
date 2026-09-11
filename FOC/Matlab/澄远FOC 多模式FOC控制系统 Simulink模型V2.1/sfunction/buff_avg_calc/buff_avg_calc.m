defs = [];

% sfun_GetBuffAvgVal
def = legacy_code('initialize');
def.SFunctionName = 'sfun_GetBuffAvgVal';
def.OutputFcnSpec = 'int32 y1 = GetBuffAvgVal(int32 u1)';
def.HeaderFiles   = {'buff_avg_calc.h'};
def.SourceFiles   = {'buff_avg_calc.c'};
defs = [defs; def];

legacy_code('sfcn_cmex_generate', defs);
legacy_code('compile', defs);
legacy_code('slblock_generate', defs, 'GetBuffAvgVal_model');
legacy_code('sfcn_tlc_generate', defs);
