#!/usr/bin/env python3
"""Check CubeMX-generated CANRPC board prerequisites without ROS or PyYAML."""
import argparse
from pathlib import Path
import re


def require(condition, message):
    if not condition:
        raise ValueError(message)


def check_repo(repo, role):
    repo = Path(repo)
    files = list((repo / 'Core/Src').glob('*.c'))
    sources = '\n'.join(path.read_text() for path in files)
    sources = re.sub(r'/\*.*?\*/|//[^\n]*', '', sources, flags=re.S)
    main = (repo / 'Core/Src/main.c').read_text()
    header = (repo / 'Core/Inc/canrpc_project.h').read_text()
    iocs = list(repo.glob('*.ioc'))
    require(len(iocs) == 1, 'exactly one .ioc is required')
    settings = dict(line.split('=', 1) for line in iocs[0].read_text().splitlines() if '=' in line)

    def ioc(key, value):
        require(settings.get(key) == str(value), f'{iocs[0].name}: {key} must be {value}')

    def pattern(expression, message):
        require(re.search(expression, sources, re.S), message)

    def assignment(name, value):
        values = re.findall(re.escape(name) + r'\s*=\s*([^;]+);', sources)
        require(values == [str(value)], f'generated {name} must be exactly {value}, got {values}')

    def define(name):
        match = re.search(r'^#define\s+' + name + r'\s+(\d+)u?\s*$', header, re.M)
        require(match, f'missing numeric project define {name}')
        return int(match[1])

    ioc('ProjectManager.KeepUserCode', 'true')
    for key in ['RCC.FDCANFreq_Value', 'RCC.HCLKFreq_Value', 'RCC.APB1Freq_Value', 'RCC.SYSCLKFreq_VALUE']:
        ioc(key, 80000000)
    fdcan = dict(ClockDivider='FDCAN_CLOCK_DIV1', FrameFormat='FDCAN_FRAME_FD_BRS', Mode='FDCAN_MODE_NORMAL',
                 AutoRetransmission='DISABLE', TransmitPause='DISABLE', ProtocolException='DISABLE',
                 NominalPrescaler=define('CANRPC_PROJECT_NOMINAL_PRESCALER'), NominalSyncJumpWidth=1,
                 NominalTimeSeg1=15, NominalTimeSeg2=4, DataPrescaler=define('CANRPC_PROJECT_DATA_PRESCALER'),
                 DataSyncJumpWidth=1, DataTimeSeg1=15, DataTimeSeg2=4, StdFiltersNbr=0, ExtFiltersNbr=1,
                 TxFifoQueueMode='FDCAN_TX_FIFO_OPERATION')
    for key, value in fdcan.items():
        ioc('FDCAN1.' + key, value)
        assignment('hfdcan1.Init.' + key, value)
    for key, value in [('Prescaler', 79), ('Period', 999)]:
        assignment('htim6.Init.' + key, value)
        ioc('TIM6.' + ('PeriodNoDither' if key == 'Period' else key), value)
    selections = re.findall(r'FdcanClockSelection\s*=\s*([^;]+);', sources)
    require(selections and all(value == 'RCC_FDCANCLKSOURCE_PCLK1' for value in selections), 'FDCAN kernel source must be PCLK1')
    for pin, signal in [('PA11', 'FDCAN1_RX'), ('PA12', 'FDCAN1_TX')]:
        ioc(pin + '.Signal', signal)
    pattern(r'GPIO_InitStruct\.Pin\s*=\s*GPIO_PIN_11\s*\|\s*GPIO_PIN_12\s*;', 'FDCAN1 requires PA11/12')
    pattern(r'GPIO_InitStruct\.Alternate\s*=\s*GPIO_AF9_FDCAN1', 'FDCAN1 GPIO alternate function mismatch')
    irqs = ['FDCAN1_IT0_IRQn', 'FDCAN1_IT1_IRQn', 'TIM6_DAC_IRQn']
    if role == 'arm':
        irqs += ['FDCAN3_IT0_IRQn', 'FDCAN3_IT1_IRQn', 'EXTI0_IRQn', 'EXTI1_IRQn']
        for pin, signal in [('PA0', 'GPIO_Input'), ('PC0', 'GPXTI0'), ('PC1', 'GPXTI1'), ('PA8', 'FDCAN3_RX'), ('PA15', 'FDCAN3_TX')]:
            ioc(pin + '.Signal', signal)
        for key, value in dict(ClockDivider='FDCAN_CLOCK_DIV1', FrameFormat='FDCAN_FRAME_FD_BRS', Mode='FDCAN_MODE_NORMAL',
                               AutoRetransmission='DISABLE', NominalPrescaler=4, NominalSyncJumpWidth=1,
                               NominalTimeSeg1=15, NominalTimeSeg2=4, ExtFiltersNbr=1).items():
            if 'FDCAN3.' + key in settings:
                ioc('FDCAN3.' + key, value)
            assignment('hfdcan3.Init.' + key, value)
        pattern(r'GPIO_InitStruct\.Alternate\s*=\s*GPIO_AF11_FDCAN3', 'FDCAN3 GPIO alternate function mismatch')
        pattern(r'GPIO_InitStruct.Pin\s*=\s*limit_sw_0_Pin\|limit_sw_1_Pin;\s*GPIO_InitStruct.Mode\s*=\s*GPIO_MODE_IT_RISING;\s*GPIO_InitStruct.Pull\s*=\s*GPIO_NOPULL;', 'arm home IRQ pins changed')
        pattern(r'GPIO_InitStruct.Pin\s*=\s*limit_Pin;\s*GPIO_InitStruct.Mode\s*=\s*GPIO_MODE_INPUT;', 'arm home input changed')
    elif role == 'pump':
        for index, pin in enumerate([7, 8, 9, 10, 11, 12], 1):
            ioc(f'PC{pin}.Signal', 'GPIO_Output')
            ioc(f'PC{pin}.GPIO_Label', f'OUT{index}')
            ioc(f'PC{pin}.GPIO_PuPd', 'GPIO_PULLDOWN')
            # CubeMX may omit the default Low level in .ioc files.
            if f'PC{pin}.PinState' in settings:
                ioc(f'PC{pin}.PinState', 'GPIO_PIN_RESET')
        pattern(r'GPIO_InitStruct.Pin\s*=\s*OUT1_Pin\|OUT2_Pin\|OUT3_Pin\|OUT4_Pin\s*\|OUT5_Pin\|OUT6_Pin;\s*GPIO_InitStruct.Mode\s*=\s*GPIO_MODE_OUTPUT_PP;\s*GPIO_InitStruct.Pull\s*=\s*GPIO_PULLDOWN;', 'pump output GPIO mode/pull changed')
        pattern(r'HAL_GPIO_WritePin\(GPIOC,\s*OUT1_Pin\|OUT2_Pin\|OUT3_Pin\|OUT4_Pin\s*\|OUT5_Pin\|OUT6_Pin,\s*GPIO_PIN_RESET\)', 'pump outputs must initialize Low (wire mask63, configured logical OFF)')
    elif role == 'pressure':
        irqs += ['DMA1_Channel1_IRQn']
        for key, value in dict(NbrOfConversion=3, ContinuousConvMode='ENABLE', DMAContinuousRequests='ENABLE', EOCSelection='ADC_EOC_SEQ_CONV').items():
            ioc('ADC2.' + key, value)
            assignment('hadc2.Init.' + key, value)
        for rank, channel in enumerate([17, 13, 3]):
            ioc(f'ADC2.Channel-{rank}\\#ChannelRegularConversion', f'ADC_CHANNEL_{channel}')
            ioc(f'ADC2.Rank-{rank}\\#ChannelRegularConversion', rank + 1)
            ioc(f'PA{rank+4}.Signal', f'ADC2_IN{channel}')
            pattern(r'sConfig.Channel\s*=\s*ADC_CHANNEL_' + str(channel) + r';\s*sConfig.Rank\s*=\s*ADC_REGULAR_RANK_' + str(rank+1), 'ADC channel/rank changed')
        ioc('Dma.ADC2.0.Mode', 'DMA_CIRCULAR')
        assignment('hdma_adc2.Init.Mode', 'DMA_CIRCULAR')
        for key, value in dict(ScanConvMode='ADC_SCAN_ENABLE', Resolution='ADC_RESOLUTION_12B', DataAlign='ADC_DATAALIGN_RIGHT').items():
            assignment('hadc2.Init.' + key, value)
            if 'ADC2.' + key in settings:
                ioc('ADC2.' + key, value)
        for key, value in dict(Direction='DMA_PERIPH_TO_MEMORY', PeriphInc='DMA_PINC_DISABLE', MemInc='DMA_MINC_ENABLE',
                               PeriphDataAlignment='DMA_PDATAALIGN_WORD', MemDataAlignment='DMA_MDATAALIGN_WORD').items():
            assignment('hdma_adc2.Init.' + key, value)
            if 'Dma.ADC2.0.' + key in settings:
                ioc('Dma.ADC2.0.' + key, value)
    for irq in irqs:
        require(settings.get('NVIC.' + irq, '').startswith('true\\:0\\:0\\:'), f'{irq} must be enabled at priority zero')
        pattern(r'HAL_NVIC_EnableIRQ\(\s*' + irq + r'\s*\)', irq + ' not enabled in generated code')
        pattern(r'HAL_NVIC_SetPriority\(\s*' + irq + r'\s*,\s*0\s*,\s*0\s*\)', irq + ' priority mismatch')
        handler = re.search(r'void\s+' + irq.replace('_IRQn', '_IRQHandler') + r'\s*\(void\)\s*\{(.*?)^}', sources, re.S | re.M)
        require(handler, irq + ' handler missing')
        if irq.startswith('FDCAN'):
            dispatch = 'HAL_FDCAN_IRQHandler(&hfdcan' + irq[5] + ')'
        elif irq.startswith('TIM6'):
            dispatch = 'HAL_TIM_IRQHandler(&htim6)'
        elif irq.startswith('DMA'):
            dispatch = 'HAL_DMA_IRQHandler(&hdma_adc2)'
        else:
            dispatch = 'HAL_GPIO_EXTI_IRQHandler(limit_sw_' + irq[4] + '_Pin)'
        require(re.sub(r'\s+', '', dispatch) in re.sub(r'\s+', '', handler[1]), irq + ' HAL dispatch missing')
    user_blocks = re.findall(r'/\* USER CODE BEGIN ([^*]+)\*/(.*?)/\* USER CODE END \1\*/', main, re.S)
    user = '\n'.join(content for _, content in user_blocks)
    user = re.sub(r'/\*.*?\*/|//[^\n]*', '', user, flags=re.S)
    for include in ['canrpc_stm32.h', 'canrpc_board.h']:
        require(f'#include "{include}"' in user, f'{include} must be in a USER CODE section')
    outside = re.sub(r'/\* USER CODE BEGIN ([^*]+)\*/.*?/\* USER CODE END \1\*/', '', main, flags=re.S)
    require(not re.search(r'#include\s+[<"](?:canrpc\w*\.h|math\.h)', outside), 'custom includes outside USER CODE')
    for call in ['rpc_init', 'rpc_poll', 'canrpc_board_valid']:
        require(re.search(r'\b' + call + r'\s*\(', user), f'{call} hook missing from USER CODE')
    require(re.search(r'canrpc_board_valid\(&hfdcan1,\s*0\)', user) and
            re.search(r'canrpc_board_valid\(&hfdcan1,\s*1\)', user), 'both transport guard stages are required')
    initialization = next((content for name, content in user_blocks if name.strip() == '2'), '')
    initialization = re.sub(r'/\*.*?\*/|//[^\n]*', '', initialization, flags=re.S)
    require(re.search(r'canrpc_board_valid\(&hfdcan1,\s*0\).*?rpc_init\(&hfdcan1,.*?canrpc_board_valid\(&hfdcan1,\s*1\).*?rpc_poll\(', initialization, re.S), 'startup guard order/USER2 placement changed')
    startup_hooks = {
        'arm': ['arm_board_config_valid(', 'motor_CAN_RxTxSettings_init(', 'arm_boot_initialize('],
        'pump': ['rpc_ready('],
        'pressure': ['HAL_ADCEx_Calibration_Start(', 'HAL_ADC_Start_DMA(', 'ADC_FLAG_EOS', 'rpc_ready('],
    }
    compact = re.sub(r'\s+', '', initialization)
    for hook in startup_hooks[role]:
        require(hook in compact, 'application startup hook missing: ' + hook)
    if role == 'pump':
        require(not re.search(r'\b(?:setPumpState|setSolenoState)\s*\(', initialization),
                'pump startup must not override the six OFF outputs')
    build = (repo / 'CMakeLists.txt').read_text()
    require('add_custom_target(canrpc_configuration_check ALL' in build and
            'add_dependencies(${CMAKE_PROJECT_NAME} canrpc_configuration_check)' in build,
            'top-level CMake must retain the per-build checker dependency')
    return True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repo', type=Path, default=Path.cwd())
    parser.add_argument('--role', required=True, choices=['arm', 'pump', 'pressure'])
    args = parser.parse_args()
    try:
        check_repo(args.repo, args.role)
    except (ValueError, OSError) as error:
        parser.exit(1, str(error) + '\n')
    print('CubeMX CANRPC prerequisites verified: ' + args.role)


if __name__ == '__main__':
    main()
