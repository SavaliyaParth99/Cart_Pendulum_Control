% create legacy mex function for cart control
def = legacy_code('initialize');
def.SourceFiles = {'spctrl.c','Fixedpt_Position_Control.c'};
def.HeaderFiles = {'xil_types.h','Fixedpt_Position_Control.h'};
def.SFunctionName = 'ex_sfun_spctrl';
% s16 spsctrl(s16 reset, s16 ee);
def.OutputFcnSpec = 'int16 y1 = spctrl(int16 u1, int32 u2, int16 u3, int32 u4)';
legacy_code('sfcn_cmex_generate', def)
legacy_code('compile', def)
%legacy_code('slblock_generate', def)
disp('That`s all folks.')
