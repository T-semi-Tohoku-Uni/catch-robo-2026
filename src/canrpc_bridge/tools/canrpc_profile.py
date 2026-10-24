#!/usr/bin/env python3
"""Validate a profile and synchronize standalone STM32 application vendors."""
import argparse
from pathlib import Path
import re
import sys
import yaml


def _integer(value, low, high, label):
    if type(value) is not int or not low <= value <= high:
        raise ValueError(f'{label} must be an integer in [{low},{high}]')
    return value


def _mapping(value, label, allowed, required):
    if not isinstance(value, dict) or set(value) - set(allowed) or set(required) - set(value):
        raise ValueError(f'{label}: invalid or missing keys')
    return value


def validate_profile(path):
    """Return a validated generic profile; no STM32 hardware assumptions apply."""
    class UniqueLoader(yaml.SafeLoader):
        pass
    def unique_mapping(loader, node, deep=False):
        result = {}
        for key_node, value_node in node.value:
            key = loader.construct_object(key_node, deep=deep)
            if key in result:
                raise ValueError('duplicate YAML key: ' + str(key))
            result[key] = loader.construct_object(value_node, deep=deep)
        return result
    UniqueLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, unique_mapping)
    p = yaml.load(Path(path).read_text(), Loader=UniqueLoader)
    _mapping(p, 'profile', ['schema_version', 'project', 'local_node', 'bus', 'timing', 'reset', 'nodes', 'routes', 'rpc', 'rpc_endpoints', 'application'],
             ['schema_version', 'project', 'local_node', 'bus', 'timing', 'reset', 'nodes', 'routes'])
    if p['schema_version'] != 1 or type(p['schema_version']) is not int:
        raise ValueError('schema_version must be 1')
    if not isinstance(p['project'], str) or not p['project'].strip():
        raise ValueError('project must be a nonempty string')
    _integer(p['local_node'], 1, 126, 'local_node')
    b = _mapping(p['bus'], 'bus', ['interface', 'nominal_bitrate', 'data_bitrate', 'configure', 'brs', 'non_blocking', 'tx_retry_timeout_ms'],
                 ['interface', 'nominal_bitrate', 'data_bitrate', 'configure', 'brs', 'non_blocking', 'tx_retry_timeout_ms'])
    if not isinstance(b['interface'], str) or not re.fullmatch(r'[A-Za-z0-9_.-]{1,15}', b['interface']):
        raise ValueError('bus.interface is invalid')
    for key in ['configure', 'brs', 'non_blocking']:
        if type(b[key]) is not bool:
            raise ValueError(f'bus.{key} must be boolean')
    _integer(b['nominal_bitrate'], 1, 1000000, 'bus.nominal_bitrate')
    _integer(b['data_bitrate'], b['nominal_bitrate'], 8000000, 'bus.data_bitrate')
    _integer(b['tx_retry_timeout_ms'], 1, 1000, 'bus.tx_retry_timeout_ms')
    t = _mapping(p['timing'], 'timing', ['priority', 'announce_priority', 'announce_period_ms', 'peer_timeout_ms', 'poll_period_ms', 'rx_budget'],
                 ['priority', 'announce_priority', 'announce_period_ms', 'peer_timeout_ms', 'poll_period_ms', 'rx_budget'])
    for key in ['priority', 'announce_priority']:
        _integer(t[key], 0, 7, 'timing.' + key)
    for key in ['announce_period_ms', 'peer_timeout_ms', 'poll_period_ms']:
        _integer(t[key], 1, 2147483647, 'timing.' + key)
    if t['peer_timeout_ms'] <= t['announce_period_ms'] or t['poll_period_ms'] > t['announce_period_ms']:
        raise ValueError('poll <= announce < peer_timeout is required')
    _integer(t['rx_budget'], 1, 65536, 'timing.rx_budget')
    r = _mapping(p['reset'], 'reset', ['targets', 'timeout_sec', 'request_window_ms', 'max_window_ms', 'retry_interval_ms', 'retry_window_ms', 'execution_timeout_ms'],
                 ['targets', 'timeout_sec', 'request_window_ms', 'max_window_ms', 'retry_interval_ms', 'retry_window_ms', 'execution_timeout_ms'])
    if type(r['timeout_sec']) not in (int, float) or not 0 < r['timeout_sec'] < 2147483647 / 1000:
        raise ValueError('reset.timeout_sec must convert to a positive signed-32-bit millisecond duration')
    for key in ['request_window_ms', 'max_window_ms', 'retry_interval_ms', 'retry_window_ms', 'execution_timeout_ms']:
        _integer(r[key], 1, 2147483647, 'reset.' + key)
    if not (r['retry_interval_ms'] <= r['retry_window_ms'] < r['request_window_ms'] <= r['max_window_ms']):
        raise ValueError('retry_interval <= retry_window < request_window <= max_window is required')
    if r['execution_timeout_ms'] > r['request_window_ms'] or r['timeout_sec'] * 1000 <= r['request_window_ms']:
        raise ValueError('execution_timeout <= request_window < startup timeout is required')
    if not isinstance(p['nodes'], dict) or not p['nodes']:
        raise ValueError('nodes must be a nonempty mapping')
    ids = []
    for name, node in p['nodes'].items():
        if not isinstance(name, str) or not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*', name):
            raise ValueError('node names must be nonempty strings')
        _mapping(node, 'node.' + name, ['id', 'reset_sources', 'announce_priority', 'rpc'], ['id', 'reset_sources'])
        ids.append(_integer(node['id'], 1, 126, 'node.' + name + '.id'))
        if 'announce_priority' in node:
            _integer(node['announce_priority'], 0, 7, 'node.' + name + '.announce_priority')
    if len(ids) != len(set(ids)) or p['local_node'] not in ids:
        raise ValueError('node IDs must be unique and include local_node')
    def node_list(values, label):
        if not isinstance(values, list):
            raise ValueError(label + ' must be a unique node list')
        for value in values:
            _integer(value, 1, 126, label)
            if value not in ids:
                raise ValueError(label + ': unknown node')
        if len(values) != len(set(values)):
            raise ValueError(label + ' must be unique')
    node_list(r['targets'], 'reset.targets')
    if p['local_node'] in r['targets']:
        raise ValueError('reset.targets must exclude local_node')
    for name, node in p['nodes'].items():
        node_list(node['reset_sources'], 'node.' + name + '.reset_sources')
        if node['id'] == p['local_node'] and node['reset_sources']:
            raise ValueError('the ROS local node does not implement a reset server')
        if node['id'] in node['reset_sources']:
            raise ValueError('reset_sources must exclude the local node')
    if not isinstance(p['routes'], list):
        raise ValueError('routes must be a list')
    names, topics, tx_subjects, rx_subjects = set(), set(), set(), set()
    subjects = {}
    for route in p['routes']:
        _mapping(route, 'route', ['name', 'direction', 'topic', 'subject', 'type', 'count', 'priority', 'source', 'require_peer', 'finite', 'min', 'max'],
                 ['name', 'direction', 'topic', 'subject', 'type', 'count'])
        if not isinstance(route['name'], str) or not route['name'].strip() or route['name'] in names:
            raise ValueError('route names must be unique nonempty strings')
        names.add(route['name'])
        _integer(route['subject'], 0, 4095, 'route.subject')
        if route['direction'] not in ('tx', 'rx') or route['type'] not in ('float32', 'int32', 'uint8'):
            raise ValueError('unsupported route direction/type')
        if not isinstance(route['topic'], str) or not re.fullmatch(r'(~/|/)?[A-Za-z_][A-Za-z0-9_]*(/[A-Za-z_][A-Za-z0-9_]*)*', route['topic']):
            raise ValueError('route.topic must be nonempty')
        topic_key = (route['direction'], route['topic'])
        if topic_key in topics:
            raise ValueError('duplicate topic in same direction')
        topics.add(topic_key)
        _integer(route['count'], 1, 64 if route['type'] == 'uint8' else 16, 'route.count')
        _integer(route.get('priority', t['priority']), 0, 7, 'route.priority')
        if route['direction'] == 'rx' and 'source' not in route:
            raise ValueError('rx route.source is required')
        if 'source' in route:
            _integer(route['source'], 1, 126, 'route.source')
            if route['source'] not in ids or route['source'] == p['local_node'] or route['direction'] == 'tx':
                raise ValueError('route.source must reference its publisher node')
        if route['direction'] == 'tx':
            if route.get('require_peer', False) or route['subject'] in tx_subjects:
                raise ValueError('TX peer gate or duplicate subject is invalid')
            tx_subjects.add(route['subject'])
        else:
            pair = (route['source'], route['subject'])
            if pair in rx_subjects:
                raise ValueError('ambiguous RX source/subject')
            rx_subjects.add(pair)
        signature = (route['type'], route['count'])
        if route['subject'] in subjects and subjects[route['subject']] != signature:
            raise ValueError('Subject has conflicting type/count')
        subjects[route['subject']] = signature
        if 'finite' in route and route['type'] != 'float32':
            raise ValueError('finite applies only to float32')
        for key in ['require_peer', 'finite']:
            if key in route and type(route[key]) is not bool:
                raise ValueError('route.' + key + ' must be boolean')
        for key in ['min', 'max']:
            if key in route:
                import math
                if type(route[key]) not in (int, float) or not math.isfinite(route[key]):
                    raise ValueError('route.' + key + ' must be finite numeric')
        if route['type'] != 'float32':
            low, high = (0,255) if route['type'] == 'uint8' else (-2147483648,2147483647)
            for key in ['min', 'max']:
                if key in route and not (low <= route[key] <= high and int(route[key]) == route[key]):
                    raise ValueError('integer bounds are out of range')
        if 'min' in route and 'max' in route and route['min'] > route['max']:
            raise ValueError('route.min must not exceed max')
    if ('rpc' in p) != ('rpc_endpoints' in p):
        raise ValueError('rpc and rpc_endpoints must be provided together')
    if 'rpc' in p:
        keys = ['ros_service','retry_interval_ms','retry_window_ms','query_interval_ms','response_timeout_ms',
                'result_retention_ms','tag_guard_ms','max_frame_age_ms','cache_capacity']
        rpc = _mapping(p['rpc'],'rpc',keys,keys)
        if not isinstance(rpc['ros_service'],str) or not re.fullmatch(r'(~/|/)?[A-Za-z_][A-Za-z0-9_]*(/[A-Za-z_][A-Za-z0-9_]*)*',rpc['ros_service']):
            raise ValueError('invalid rpc.ros_service')
        for key in keys[1:-1]:
            _integer(rpc[key],1,2147483647,'rpc.'+key)
        _integer(rpc['cache_capacity'],1,32,'rpc.cache_capacity')
        if not rpc['retry_interval_ms'] <= rpc['retry_window_ms'] < rpc['response_timeout_ms']:
            raise ValueError('invalid ordinary RPC retry window')
        if rpc['query_interval_ms'] >= rpc['response_timeout_ms']:
            raise ValueError('RPC query interval must be shorter than response timeout')
        if rpc['result_retention_ms'] < rpc['response_timeout_ms'] + rpc['max_frame_age_ms']:
            raise ValueError('RPC result retention is too short')
        if rpc['tag_guard_ms'] < rpc['response_timeout_ms'] + rpc['result_retention_ms'] + rpc['max_frame_age_ms']:
            raise ValueError('RPC tag guard is too short')
        if not isinstance(p['rpc_endpoints'],list):
            raise ValueError('rpc_endpoints must be a list')
        names,addresses,diagnostic_nodes=set(),set(),set()
        for e in p['rpc_endpoints']:
            _mapping(e,'rpc endpoint',['name','destination','service','request_bytes','response_bytes','require_ready','startup_diagnostic'],
                     ['name','destination','service','request_bytes','response_bytes'])
            if not isinstance(e['name'],str) or not e['name'].strip() or e['name'] in names:
                raise ValueError('invalid/duplicate RPC endpoint name')
            names.add(e['name'])
            _integer(e['destination'],1,126,'endpoint.destination')
            if e['destination'] not in ids or e['destination']==p['local_node']:
                raise ValueError('RPC destination must be a declared remote node')
            _integer(e['service'],0,31,'endpoint.service')
            for key in ['request_bytes','response_bytes']:
                _integer(e[key],0,64,'endpoint.'+key)
            if 'require_ready' in e and type(e['require_ready']) is not bool:
                raise ValueError('endpoint.require_ready must be boolean')
            if 'startup_diagnostic' in e and type(e['startup_diagnostic']) is not bool:
                raise ValueError('startup_diagnostic must be boolean')
            if e.get('startup_diagnostic',False) and (e['request_bytes']!=0 or e.get('require_ready',True)):
                raise ValueError('startup diagnostic must be readonly with require_ready=false')
            if e.get('startup_diagnostic',False):
                if e['destination'] in diagnostic_nodes:
                    raise ValueError('only one startup diagnostic per destination is allowed')
                diagnostic_nodes.add(e['destination'])
            address=(e['destination'],e['service'])
            if address in addresses:
                raise ValueError('duplicate destination/service endpoint')
            addresses.add(address)
        for node in p['nodes'].values():
            if 'rpc' in node:
                override=node['rpc']
                fields=keys[1:]
                _mapping(override,'node.rpc',fields,fields)
                for key in fields[:-1]:
                    _integer(override[key],1,2147483647,'node.rpc.'+key)
                _integer(override['cache_capacity'],1,32,'node.rpc.cache_capacity')
                if not override['retry_interval_ms'] <= override['retry_window_ms'] < override['response_timeout_ms']:
                    raise ValueError('invalid node RPC retry window')
                if override['query_interval_ms']>=override['response_timeout_ms'] or override['result_retention_ms']<override['response_timeout_ms']+override['max_frame_age_ms']:
                    raise ValueError('invalid node RPC query/retention')
                if override['tag_guard_ms']<override['response_timeout_ms']+override['result_retention_ms']+override['max_frame_age_ms']:
                    raise ValueError('invalid node RPC guard')
    if 'application' in p and not isinstance(p['application'],dict):
        raise ValueError('application must be a mapping')
    return p


def _mcu_headers(p):
    roles = ['ros', 'arm', 'pump', 'pressure']
    if p['reset']['execution_timeout_ms'] <= 10:
        raise ValueError('STM32 reset execution timeout must exceed its 10 ms main poll')
    if any(role not in p['nodes'] for role in roles) or p['local_node'] != p['nodes']['ros']['id']:
        raise ValueError('STM32 application generation requires ros/arm/pump/pressure roles and local ros')
    routes = {r['name']: r for r in p['routes']}
    for name, kind, count, direction, source in [('current','float32',3,'rx','arm'),
                                               ('pressure','int32',3,'rx','pressure')]:
        r = routes.get(name, {})
        if (r.get('type'),r.get('count'),r.get('direction'),r.get('source',p['local_node'])) != (kind,count,direction,p['nodes'][source]['id']):
            raise ValueError('STM32 application route mismatch: ' + name)
    if 'rpc' not in p:
        raise ValueError('STM32 application requires ordinary RPC settings')
    endpoints = {e['name']:e for e in p['rpc_endpoints']}
    for name,role,req,resp in [('pump_set','pump',4,8),('arm_ready','arm',0,48),('arm_target','arm',12,20)]:
        e=endpoints.get(name,{})
        if (e.get('destination'),e.get('request_bytes'),e.get('response_bytes')) != (p['nodes'][role]['id'],req,resp):
            raise ValueError('STM32 application RPC endpoint mismatch: '+name)
        if e.get('require_ready',True) != (name!='arm_ready'):
            raise ValueError('STM32 endpoint readiness policy mismatch: '+name)
    motor_keys=['step_timeout_ms','homing_timeout_ms','feedback_timeout_ms','feedback_max_age_ms','inter_command_ms','home_settle_ms']
    motor=p.get('application',{}).get('motor_initialization',{})
    _mapping(motor,'application.motor_initialization',motor_keys,motor_keys)
    for key in motor_keys:
        _integer(motor[key],1,2147483647,'motor_initialization.'+key)
    if motor['step_timeout_ms']<=10 or motor['feedback_timeout_ms']<=10 or motor['feedback_max_age_ms']>100:
        raise ValueError('STM32 motor timing must allow main polling and respect existing ADRC 100ms freshness')
    control=p.get('application',{}).get('arm_control',{})
    _mapping(control,'application.arm_control',['native_ack_timeout_ms','target_period_ms'],['native_ack_timeout_ms'])
    if 'target_period_ms' in control:
        _integer(control['target_period_ms'],10,2147483647,'target_period_ms')
    _integer(control['native_ack_timeout_ms'],1,50,'native_ack_timeout_ms')
    if control['native_ack_timeout_ms']+10>=p['nodes']['arm'].get('rpc',p['rpc'])['response_timeout_ms']:
        raise ValueError('native ACK timeout plus 10 ms main poll must be shorter than arm RPC response timeout')
    prescalers = []
    for key, limit in [('nominal_bitrate',512), ('data_bitrate',32)]:
        rate = p['bus'][key]
        if 80000000 % (rate * 20) or not 1 <= 80000000 // (rate * 20) <= limit:
            raise ValueError('STM32 80MHz/20tq cannot represent bus.' + key)
        prescalers.append(80000000 // (rate * 20))
    result = {}
    for role in roles[1:]:
        values = {}
        for name in roles:
            values['CANRPC_NODE_' + name.upper()] = p['nodes'][name]['id']
        for name, route in routes.items():
            if name in ['target','current','pump','pressure']:
                values['CANRPC_SUBJECT_' + name.upper()] = route['subject']
                values['CANRPC_PROJECT_' + name.upper() + '_PRIORITY'] = route.get('priority', p['timing']['priority'])
        values.update(CANRPC_PROJECT_PRIORITY=p['timing']['priority'],
                      CANRPC_PROJECT_ANNOUNCE_PRIORITY=p['nodes'][role].get('announce_priority',p['timing']['announce_priority']),
                      CANRPC_PROJECT_ANNOUNCE_PERIOD_MS=p['timing']['announce_period_ms'],
                      CANRPC_PROJECT_PEER_TIMEOUT_MS=p['timing']['peer_timeout_ms'],
                      CANRPC_PROJECT_RESET_MAX_WINDOW_MS=p['reset']['max_window_ms'],
                      CANRPC_PROJECT_RESET_EXECUTE_TIMEOUT_MS=p['reset']['execution_timeout_ms'],
                      CANRPC_PROJECT_RESET_ENABLED=int(bool(p['nodes'][role]['reset_sources'])),
                      CANRPC_PROJECT_RESET_SOURCE_COUNT=len(p['nodes'][role]['reset_sources']),
                      CANRPC_PROJECT_BRS=int(p['bus']['brs']),
                      CANRPC_PROJECT_NOMINAL_PRESCALER=prescalers[0],CANRPC_PROJECT_DATA_PRESCALER=prescalers[1])
        for key,value in {**p['rpc'],**p['nodes'][role].get('rpc',{})}.items():
            if key != 'ros_service':
                values['CANRPC_PROJECT_RPC_'+key.upper()]=value
        endpoint=endpoints.get('arm_ready' if role=='arm' else 'pump_set') if role!='pressure' else None
        values['CANRPC_PROJECT_RPC_ENABLED']=int(endpoint is not None)
        values['CANRPC_PROJECT_RPC_SERVICE']=endpoint['service'] if endpoint else 0
        values['CANRPC_PROJECT_RPC_REQUEST_BYTES']=endpoint['request_bytes'] if endpoint else 0
        values['CANRPC_PROJECT_RPC_RESPONSE_BYTES']=endpoint['response_bytes'] if endpoint else 0
        values['CANRPC_PROJECT_RPC_REQUIRE_READY']=int(endpoint.get('require_ready',True)) if endpoint else 1
        self_endpoints=[e for e in p['rpc_endpoints'] if e['destination']==p['nodes'][role]['id']]
        values['CANRPC_PROJECT_RPC_ENDPOINT_COUNT']=len(self_endpoints)
        values['CANRPC_PROJECT_NATIVE_ACK_TIMEOUT_MS']=control['native_ack_timeout_ms']
        for name in ['arm_ready','arm_target','pump_set']:
            if name in endpoints:
                values['CANRPC_PROJECT_'+name.upper()+'_SERVICE']=endpoints[name]['service']
        for key,value in motor.items():
            values['CANRPC_PROJECT_MOTOR_'+key.upper()]=value
        lines = ['#ifndef CANRPC_PROJECT_H', '#define CANRPC_PROJECT_H', '', '/* Generated by tools/canrpc_profile.py; edit the YAML profile. */']
        lines += [f'#define {key} {value}u' for key,value in values.items()]
        wire=lambda n:n if n<=8 else next(v for v in [12,16,20,24,32,48,64] if v>=n)
        descriptors=[f"{{{e['service']}u,{e['request_bytes']}u,{e['response_bytes']}u,{int(e.get('require_ready',True))}u}}" for e in self_endpoints]
        lines+=['#define CANRPC_PROJECT_RPC_ENDPOINTS {'+', '.join(descriptors or ['{0u,0u,0u,1u}'])+'}']
        sources = p['nodes'][role]['reset_sources'] or [0]
        lines += ['#define CANRPC_PROJECT_RESET_SOURCES {' + ', '.join(str(x)+'u' for x in sources) + '}', '#endif', '']
        result[role] = '\n'.join(lines)
    return result, prescalers


def sync_mcu(profile_path, mcu_root, check=False):
    p = validate_profile(profile_path)
    headers, prescalers = _mcu_headers(p)
    package = Path(__file__).resolve().parents[1]
    core = (package/'core/include/canrpc_core.h').read_bytes()
    adapter = (package/'adapters/stm32g4/canrpc_stm32.h').read_bytes()
    wrapper = b'#ifndef CANRPC_APPLICATION_H\n#define CANRPC_APPLICATION_H\n#include "canrpc_core.h"\n#include "canrpc_project.h"\n#endif\n'
    mismatches = []
    all_outputs = {}
    for role, repo_name in [('arm','catch2026_robstride'),('pump','catchrobo2026_pump'),('pressure','catch2026_pressure')]:
        repo = Path(mcu_root)/repo_name
        init_source = repo/'Core/Src/fdcan.c' if role == 'pressure' else repo/'Core/Src/main.c'
        init_text = init_source.read_text()
        expected_init = {'NominalPrescaler':'CANRPC_PROJECT_NOMINAL_PRESCALER',
                         'DataPrescaler':'CANRPC_PROJECT_DATA_PRESCALER',
                         'NominalTimeSeg1':'15', 'NominalTimeSeg2':'4',
                         'DataTimeSeg1':'15', 'DataTimeSeg2':'4'}
        for field, value in expected_init.items():
            if not re.search(r'hfdcan1\.Init\.'+field+r'\s*=\s*'+value+r'\s*;',init_text):
                raise ValueError(str(init_source) + ': unexpected FDCAN1.' + field)
        outputs = {repo/'Core/Inc/canrpc_core.h':core, repo/'Core/Inc/canrpc_stm32.h':adapter, repo/'Core/Inc/canrpc_project.h':headers[role].encode(), repo/'Core/Inc/canrpc.h':wrapper}
        for ioc in repo.glob('*.ioc'):
            text = ioc.read_text()
            if not re.search(r'^RCC.FDCANFreq_Value=80000000$',text,re.M):
                raise ValueError(str(ioc) + ': expected 80 MHz FDCAN clock')
            for key,value in [('NominalPrescaler',prescalers[0]),('DataPrescaler',prescalers[1]),('StdFiltersNbr',0),('ExtFiltersNbr',1)]:
                text,n = re.subn(r'^FDCAN1\.'+key+r'=.*$',f'FDCAN1.{key}={value}',text,flags=re.M)
                if n != 1:
                    raise ValueError(f'{ioc}: missing FDCAN1.{key}')
            outputs[ioc] = text.encode()
        all_outputs.update(outputs)
    for path, content in all_outputs.items():
        if not path.exists() or path.read_bytes() != content:
            mismatches.append(str(path))
    if not check:
        for path, content in all_outputs.items():
            if str(path) in mismatches:
                path.write_bytes(content)
    if check and mismatches:
        raise ValueError('outdated MCU vendors/config: ' + ', '.join(mismatches))
    return p


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('profile', type=Path)
    parser.add_argument('--mcu-root', type=Path)
    parser.add_argument('--check', action='store_true')
    args = parser.parse_args()
    try:
        if args.mcu_root:
            sync_mcu(args.profile,args.mcu_root,args.check)
        else:
            validate_profile(args.profile)
    except (OSError, ValueError, yaml.YAMLError) as error:
        parser.exit(1,str(error)+'\n')
    print('CANRPC profile validated' + (' and MCU vendors synchronized' if args.mcu_root else ''))


if __name__ == '__main__':
    main()
