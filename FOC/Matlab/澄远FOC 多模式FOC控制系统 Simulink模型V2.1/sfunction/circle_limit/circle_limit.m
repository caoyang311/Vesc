defs = [];

%ccode_lct_src = '.\c_code';

% sfun_EmSvpwm
def = legacy_code('initialize');
def.SFunctionName = 'sfun_CircleLimit';
def.OutputFcnSpec = 'void CircleLimit(int16 u1, int16 u2, int16 y1[1], int16 y2[1])';
def.HeaderFiles   = {'circle_limit.h'};
def.SourceFiles   = {'circle_limit.c'};
defs = [defs; def];

legacy_code('sfcn_cmex_generate', defs);
legacy_code('compile', defs);
legacy_code('slblock_generate', defs, 'circle_limit_model');
legacy_code('sfcn_tlc_generate', defs);
