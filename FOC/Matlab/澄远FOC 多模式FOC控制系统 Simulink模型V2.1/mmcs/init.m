% 任务1 start
% 从'trajectory_ctrl.h'导入C定义的PosCtrlHandle_t数据类型，供simulink模型使用
% Simulink.importExternalCTypes('trajectory_ctrl.h');
% 任务1 end


% 任务2 start
% 根据选择的电机类型，修改数据字典中相关的预定义数据

% 获取当前模型的数据字典对象、设计数据分区对象
dictionaryObj = Simulink.data.dictionary.open('mmcs_s2f_dd.sldd');
dDataSectObj = getSection(dictionaryObj,'Design Data');

% 获取相关数据条目对象，读取预定义的电机类型
entryObj = getEntry(dDataSectObj, 'MOTOR_TYPE_TG5P60');
MOTOR_TYPE_TG5P60_param = getValue(entryObj);
entryObj = getEntry(dDataSectObj, 'MOTOR_TYPE_TB2P');
MOTOR_TYPE_TB2P_param = getValue(entryObj);
entryObj = getEntry(dDataSectObj, 'MOTOR_TYPE_JSF630');
MOTOR_TYPE_JSF630_param = getValue(entryObj);
entryObj = getEntry(dDataSectObj, 'MOTOR_TYPE');
MOTOR_TYPE_param = getValue(entryObj);

% 获取需要修改的数据条目对象
entryObj = getEntry(dDataSectObj, 'MOTOR_PNF');
MOTOR_PNF_param = getValue(entryObj);
entryObj = getEntry(dDataSectObj, 'MOTOR_PN');
MOTOR_PN_param = getValue(entryObj);
entryObj = getEntry(dDataSectObj, 'MOTOR_RS');
MOTOR_RS_param = getValue(entryObj);
entryObj = getEntry(dDataSectObj, 'MOTOR_LS');
MOTOR_LS_param = getValue(entryObj);
entryObj = getEntry(dDataSectObj, 'MOTOR_KE');
MOTOR_KE_param = getValue(entryObj);
entryObj = getEntry(dDataSectObj, 'MOTOR_J');
MOTOR_J_param = getValue(entryObj);
entryObj = getEntry(dDataSectObj, 'MOTOR_F');
MOTOR_F_param = getValue(entryObj);
entryObj = getEntry(dDataSectObj, 'MOTOR_TL');
MOTOR_TL_param = getValue(entryObj);

entryObj = getEntry(dDataSectObj, 'ENC_PULSE_NUM');
ENC_PULSE_NUM_param = getValue(entryObj);
entryObj = getEntry(dDataSectObj, 'U32MAXdivPulseNumber');
U32MAXdivPulseNumber_param = getValue(entryObj);

entryObj = getEntry(dDataSectObj, 'PI_KP_TQ');
PI_KP_TQ_param = getValue(entryObj);
entryObj = getEntry(dDataSectObj, 'PI_KI_TQ');
PI_KI_TQ_param = getValue(entryObj);

entryObj = getEntry(dDataSectObj, 'PI_KP_SPD');
PI_KP_SPD_param = getValue(entryObj);
entryObj = getEntry(dDataSectObj, 'PI_KI_SPD');
PI_KI_SPD_param = getValue(entryObj);

entryObj = getEntry(dDataSectObj, 'PI_KP_POS');
PI_KP_POS_param = getValue(entryObj);
entryObj = getEntry(dDataSectObj, 'PI_KI_POS');
PI_KI_POS_param = getValue(entryObj);

entryObj = getEntry(dDataSectObj, 'PI_KP_PLL');
PI_KP_PLL_param = getValue(entryObj);
entryObj = getEntry(dDataSectObj, 'PI_KI_PLL');
PI_KI_PLL_param = getValue(entryObj);

entryObj = getEntry(dDataSectObj, 'STO_C1');
STO_C1_param = getValue(entryObj);
entryObj = getEntry(dDataSectObj, 'STO_C2');
STO_C2_param = getValue(entryObj);
entryObj = getEntry(dDataSectObj, 'STO_C3');
STO_C3_param = getValue(entryObj);
entryObj = getEntry(dDataSectObj, 'STO_C4');
STO_C4_param = getValue(entryObj);
entryObj = getEntry(dDataSectObj, 'STO_C5');
STO_C5_param = getValue(entryObj);

% 根据电机类型设置不同的电机参数值
if MOTOR_TYPE_param.Value == MOTOR_TYPE_TG5P60_param.Value
    % 为电机TG5P60设置参数值
    % 电机相关参数
    MOTOR_PNF_param.Value = 5;          % 极对数：浮点
    MOTOR_PN_param.Value = uint8(5);    % 极对数：整形
    MOTOR_RS_param.Value = 0.27;        % 相电阻：Ω
    MOTOR_LS_param.Value = 0.00035;     % 相电感：H
    MOTOR_KE_param.Value = 6.264;       % 反电势常数：V/krpm
    MOTOR_J_param.Value = 0.000015;     % 转动惯量：kg.m^2
    MOTOR_F_param.Value = 0.000011;     % 阻尼系数：N.m.s
    MOTOR_TL_param.Value = 0.001;       % 模拟的空载转矩：N.m

    % 编码器相关参数
    ENC_PULSE_NUM_param.Value = uint16(10000);
    U32MAXdivPulseNumber_param.Value = uint32(429497);

    % 电流PI控制器参数
    PI_KP_TQ_param.Value = fi(0.39, 1, 32, 16);
    PI_KI_TQ_param.Value = fi(300.6, 1, 32, 16);

    % 速度PI控制器参数
    PI_KP_SPD_param.Value = fi(5.611, 1, 32, 16);
    PI_KI_SPD_param.Value = fi(4.189, 1, 32, 16);

    % 位置PI控制器参数
    PI_KP_POS_param.Value = fi(1, 1, 32, 16);
    PI_KI_POS_param.Value = fi(1, 1, 32, 16);

    % PLL PI控制器参数
    PI_KP_PLL_param.Value = fi(0.0687, 1, 32, 16);
    PI_KI_PLL_param.Value = fi(0.015, 1, 32, 16);
    
    % 观测器相关参数
    STO_C1_param.Value = int32(316);
    STO_C2_param.Value = int32(-5907);
    STO_C3_param.Value = int32(1524);
    STO_C4_param.Value = int32(8271);
    STO_C5_param.Value = int32(1622);
    
elseif MOTOR_TYPE_param.Value == MOTOR_TYPE_TB2P_param.Value
    % 为电机TB2P设置参数值
    % 电机相关参数
    MOTOR_PNF_param.Value = 2;          % 极对数：浮点
    MOTOR_PN_param.Value = uint8(2);    % 极对数：整形
    MOTOR_RS_param.Value = 0.575;       % 相电阻：Ω
    MOTOR_LS_param.Value = 0.00105;     % 相电感：H
    MOTOR_KE_param.Value = 5.96708;     % 反电势常数：V/krpm
    MOTOR_J_param.Value = 7.5E-6;       % 转动惯量：kg.m^2
    MOTOR_F_param.Value = 6.3E-6;       % 阻尼系数：N.m.s
    MOTOR_TL_param.Value = 0.001;       % 模拟的空载转矩：N.m

    % 编码器相关参数
    ENC_PULSE_NUM_param.Value = uint16(10000);
    U32MAXdivPulseNumber_param.Value = uint32(429497);

    % 电流PI控制器参数
    PI_KP_TQ_param.Value = fi(0.83, 1, 32, 16);
    PI_KI_TQ_param.Value = fi(454.5, 1, 32, 16);

    % 速度PI控制器参数
    PI_KP_SPD_param.Value = fi(1.365, 1, 32, 16);
    PI_KI_SPD_param.Value = fi(1.147, 1, 32, 16);

    % 位置PI控制器参数
    PI_KP_POS_param.Value = fi(0.39, 1, 32, 16);
    PI_KI_POS_param.Value = fi(0, 1, 32, 16);

    % PLL PI控制器参数
    PI_KP_PLL_param.Value = fi(0.03, 1, 32, 16);
    PI_KI_PLL_param.Value = fi(0.015, 1, 32, 16);

    % 观测器相关参数
    STO_C1_param.Value = int32(224);
    STO_C2_param.Value = int32(-5976);
    STO_C3_param.Value = int32(484);
    STO_C4_param.Value = int32(24634);
    STO_C5_param.Value = int32(541);

elseif MOTOR_TYPE_param.Value == MOTOR_TYPE_JSF630_param.Value
    % 为电机TG5P40设置参数值
    % 电机相关参数
    MOTOR_PNF_param.Value = 4;          % 极对数：浮点
    MOTOR_PN_param.Value = uint8(4);    % 极对数：整形
    MOTOR_RS_param.Value = 0.445;       % 相电阻：Ω
    MOTOR_LS_param.Value = 0.00031;     % 相电感：H
    MOTOR_KE_param.Value = 5.656;       % 反电势常数：V/krpm
    MOTOR_J_param.Value = 0.0000028;    % 转动惯量：kg.m^2
    MOTOR_F_param.Value = 0.000007;     % 阻尼系数：N.m.s
    MOTOR_TL_param.Value = 0.001;       % 模拟的空载转矩：N.m
    
    % 编码器相关参数
    ENC_PULSE_NUM_param.Value = uint16(4000);
    U32MAXdivPulseNumber_param.Value = uint32(1073741);

    % 电流PI控制器参数
    PI_KP_TQ_param.Value = fi(0.64, 1, 32, 16);
    PI_KI_TQ_param.Value = fi(922, 1, 32, 16);

    % 速度PI控制器参数
    PI_KP_SPD_param.Value = fi(2.073, 1, 32, 16);
    PI_KI_SPD_param.Value = fi(5.183, 1, 32, 16);

    % 位置PI控制器参数
    PI_KP_POS_param.Value = fi(1, 1, 32, 16);
    PI_KI_POS_param.Value = fi(1, 1, 32, 16);

    % PLL PI控制器参数
    PI_KP_PLL_param.Value = fi(0.0718, 1, 32, 16);
    PI_KI_PLL_param.Value = fi(0.015, 1, 32, 16);    

    % 观测器相关参数
    STO_C1_param.Value = int32(588);
    STO_C2_param.Value = int32(-5703);
    STO_C3_param.Value = int32(1554);
    STO_C4_param.Value = int32(7484);
    STO_C5_param.Value = int32(1831);

else
    error('未知的电机类型');
end

% 获取相关数据条目对象，将上面配置的参数写入
entryObj = getEntry(dDataSectObj, 'MOTOR_PNF');
setValue(entryObj, MOTOR_PNF_param);
entryObj = getEntry(dDataSectObj, 'MOTOR_PN');
setValue(entryObj, MOTOR_PN_param);
entryObj = getEntry(dDataSectObj, 'MOTOR_RS');
setValue(entryObj, MOTOR_RS_param);
entryObj = getEntry(dDataSectObj, 'MOTOR_LS');
setValue(entryObj, MOTOR_LS_param);
entryObj = getEntry(dDataSectObj, 'MOTOR_KE');
setValue(entryObj, MOTOR_KE_param);
entryObj = getEntry(dDataSectObj, 'MOTOR_J');
setValue(entryObj, MOTOR_J_param);
entryObj = getEntry(dDataSectObj, 'MOTOR_F');
setValue(entryObj, MOTOR_F_param);
entryObj = getEntry(dDataSectObj, 'MOTOR_TL');
setValue(entryObj, MOTOR_TL_param);

entryObj = getEntry(dDataSectObj, 'ENC_PULSE_NUM');
setValue(entryObj, ENC_PULSE_NUM_param);
entryObj = getEntry(dDataSectObj, 'U32MAXdivPulseNumber');
setValue(entryObj, U32MAXdivPulseNumber_param);

entryObj = getEntry(dDataSectObj, 'PI_KP_TQ');
setValue(entryObj, PI_KP_TQ_param);
entryObj = getEntry(dDataSectObj, 'PI_KI_TQ');
setValue(entryObj, PI_KI_TQ_param);

entryObj = getEntry(dDataSectObj, 'PI_KP_SPD');
setValue(entryObj, PI_KP_SPD_param);
entryObj = getEntry(dDataSectObj, 'PI_KI_SPD');
setValue(entryObj, PI_KI_SPD_param);

entryObj = getEntry(dDataSectObj, 'PI_KP_POS');
setValue(entryObj, PI_KP_POS_param);
entryObj = getEntry(dDataSectObj, 'PI_KI_POS');
setValue(entryObj, PI_KI_POS_param);

entryObj = getEntry(dDataSectObj, 'PI_KP_PLL');
setValue(entryObj, PI_KP_PLL_param);
entryObj = getEntry(dDataSectObj, 'PI_KI_PLL');
setValue(entryObj, PI_KI_PLL_param);

entryObj = getEntry(dDataSectObj, 'STO_C1');
setValue(entryObj, STO_C1_param);
entryObj = getEntry(dDataSectObj, 'STO_C2');
setValue(entryObj, STO_C2_param);
entryObj = getEntry(dDataSectObj, 'STO_C3');
setValue(entryObj, STO_C3_param);
entryObj = getEntry(dDataSectObj, 'STO_C4');
setValue(entryObj, STO_C4_param);
entryObj = getEntry(dDataSectObj, 'STO_C5');
setValue(entryObj, STO_C5_param);

% 保存数据字典的更改
saveChanges(dictionaryObj);

% 任务2 end

