% create legacy mex function for cart control
def = legacy_code('initialize');
def.SourceFiles = {'spctrl.c','Fixedpt_Speed_Control.c'};
def.HeaderFiles = {'xil_types.h','Fixedpt_Speed_Control.h'};
def.SFunctionName = 'ex_sfun_spsctrl';
% s16 spsctrl(s16 reset, s16 ee);
def.OutputFcnSpec = 'int16 y1 = spctrl(int16 u1, int16 u2, int32 u3)';
legacy_code('sfcn_cmex_generate', def)
legacy_code('compile', def)
legacy_code('slblock_generate', def)
disp('That`s all folks.')
