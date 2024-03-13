#!/usr/bin/env python3

# import git
import glob
import os
import re
from pathlib import Path
import itertools
import xmltodict
import dataclasses
from typing import Dict, Optional
import json


def dict_factory(x):
    return {k: v for (k, v) in x if v is not None}


@dataclasses.dataclass
class RangeData:
    high: str
    low: str


@dataclasses.dataclass
class ParamData:
    DisplayName: str

    # Rarely missing
    Description: Optional[str]

    # For that required for library parameters
    User: Optional[str]

    Bitmask: Optional[Dict[str, str]] = None
    Range: Optional[RangeData] = None
    Values: Optional[Dict[str, str]] = None
    RebootRequired: Optional[str] = None
    Increment: Optional[str] = None
    Units: Optional[str] = None
    ReadOnly: Optional[str] = None
    Volatile: Optional[str] = None
    Calibration: Optional[str] = None
    path: Optional[str] = None

    def __setitem__(self, key, value):
        super().__setattr__(key, value)

    def _filterr(self, param):
        filtered = {k: v for k, v in param.items() if v is not None}
        return filtered


@dataclasses.dataclass
class JsonData:
    version: int


@dataclasses.dataclass
class JsonParam:
    json: JsonData


repository = Path(__file__).parent.parent
files = [list(path) for path in [folder.iterdir()
                                 for folder in repository.iterdir() if folder.is_dir()]]
xmls = [path for path in list(itertools.chain(*files))
        if path.suffix == '.xml']


def generate_parameter(param):
    paramData = ParamData(
        DisplayName=param['@humanName'],
        Description=param['@documentation'] if '@documentation' in param else None,
        User=param['@user'] if '@user' in param else None,
    )

    if 'values' in param:
        paramData.Values = {}
        param_value = param['values']['value']
        if type(param_value) == list:
            for value in param_value:
                paramData.Values[value['@code']] = value['#text']
        else:
            paramData.Values[param_value['@code']] = param_value['#text']

    if 'field' in param:
        if type(param['field']) == list:
            for field in param['field']:
                if field['@name'] == 'Range':
                    paramData[field['@name']] = {
                        'high': field['#text'].split(' ')[1].strip(),
                        'low': field['#text'].split(' ')[0].strip(),
                    }
                    continue
                if field['@name'] == 'Bitmask':
                    bits = field['#text'].split(',')
                    bits_result = {}
                    for bit in bits:
                        bits_result[bit.split(':')[0].strip()] = bit.split(':')[
                            1].strip()
                    paramData[field['@name']] = bits_result
                    continue
                if field['@name'] == 'Values' and ',' in field['#text']:
                    bits = field['#text'].split(',')
                    bits_result = {}
                    for bit in bits:
                        bits_result[bit.split(':')[0].strip()] = bit.split(':')[
                            1].strip()
                    paramData[field['@name']] = bits_result
                    continue
                paramData[field['@name']] = field['#text']
        else:
            field = param['field']
            if field['@name'] == 'Range':
                paramData[field['@name']] = {
                    'high': field['#text'].split(' ')[1].strip(),
                    'low': field['#text'].split(' ')[0].strip(),
                }
            elif field['@name'] == 'Bitmask':
                bits = field['#text'].split(',')
                bits_result = {}
                for bit in bits:
                    bits_result[bit.split(':')[0].strip()] = bit.split(':')[
                        1].strip()
                paramData[field['@name']] = bits_result
            elif field['@name'] == 'Values' and ',' in field['#text']:
                bits = field['#text'].split(',')
                bits_result = {}
                for bit in bits:
                    bits_result[bit.split(':')[0].strip()] = bit.split(':')[
                        1].strip()
                paramData[field['@name']] = bits_result
            else:
                # Old parameter files are wrong and there are fields without values
                if '#text' in field:
                    paramData[field['@name']] = field['#text']

    return paramData


def process(path):
    folder = path.parent

    print(f'Folder: {folder.name}')

    vehicle_type = None
    vehicle_json_type = None

    if 'Copter' in folder.name:
        vehicle_type = 'ArduCopter'
        vehicle_json_type = 'Copter'
    elif 'Plane' in folder.name:
        vehicle_type = 'ArduPlane'
        vehicle_json_type = 'Plane'
    elif 'Rover' in folder.name:
        vehicle_type = 'APMrover2'
        vehicle_json_type = 'Rover'
    elif 'Sub' in folder.name:
        vehicle_type = 'ArduSub'
        vehicle_json_type = 'Sub'

    xml = folder / 'apm.pdef.xml'
    json_path = folder / 'apm.pdef.json'

    result = dataclasses.asdict(JsonParam(
        json=JsonData(
            version=0
        )
    ))
    result[vehicle_json_type] = {}

    content = {}
    with open(xml) as fd:
        content = xmltodict.parse(fd.read())

    # check for vehicle specific parameters
    parameters = content['paramfile']['vehicles']['parameters']

    # There is a special case where it can have all vehicles on the same file
    # Get only the one that we are processing
    if type(parameters) == list:
        parameters = [
            param for param in parameters if param['@name'] == vehicle_type]
        if parameters == []:
            return
        parameters = parameters[0]

    for param in parameters['param']:
        name = param['@name'].split(':')[-1]
        result[vehicle_json_type][name] = dataclasses.asdict(
            generate_parameter(param), dict_factory=dict_factory)

    parameters = content['paramfile']['libraries']['parameters']
    for param in parameters:
        name = param['@name'].split(':')[-1]
        result[name] = {}
        if type(param['param']) == list:
            for param in param['param']:
                result[name][param['@name']] = dataclasses.asdict(
                    generate_parameter(param), dict_factory=dict_factory)
        else:
            result[name][param['param']['@name']] = dataclasses.asdict(
                generate_parameter(param['param']), dict_factory=dict_factory)

    with open(json_path, 'w') as json_fd:
        json_fd.write(json.dumps(result, indent=2, sort_keys=True))


for xml in xmls:
    process(xml)
