#!/usr/bin/env python3

import pathlib
import json


N_SERVICES = 5
RANGE_SERVICES = range(1, N_SERVICES + 1)

DIRNAME_SCRIPT = pathlib.Path(__file__).resolve().parent.name
assert DIRNAME_SCRIPT in ('coordination_oru', 'coordination_oru_for_containers'), DIR_SCRIPT
IS_FC = DIRNAME_SCRIPT == 'coordination_oru_for_containers'
SUFFIX = 'fc' if IS_FC else ''

N_SERVICES = 5
RANGE_SERVICES = range(1, N_SERVICES + 1)

def main():
    volumes = {}
    for i in RANGE_SERVICES:
        for base in 'root-gradle', 'project-gradle', 'project-build':
            volumes[f'{base}-{i}{SUFFIX}'] = None

    services = {}
    for i in RANGE_SERVICES:
        service = {
            'container_name': f'c{i}{SUFFIX}',
            'build': '.',
            'ports': [f'{(5910 if IS_FC else 5900) + i}:5900'],
            'volumes': [
                './:/coordination_oru/',
                f'project-gradle-{i}{SUFFIX}:/coordination_oru/.gradle/',
                f'project-build-{i}{SUFFIX}:/coordination_oru/build/',
                f'root-gradle-{i}{SUFFIX}:/root/.gradle/',
                '/home/olga/miniconda3/:/home/olga/miniconda3/:ro',
            ],
            'environment': {
                'WORKER': f'c{i}',
                'IS_VISUALIZATION': '',
                'RUNDIRS': '$RUNDIRS',
            },
            'command': ['nice', '-n5', 'container/index.sh'],
            'network_mode': 'bridge',  # for the Internet to work
            'shm_size': '4gb',  # for Chromium
        }
        services[f'oru-{i}{SUFFIX}'] = service

    config = {'volumes': volumes, 'services': services}

    with open('compose.yaml', 'w') as file:
        json.dump(config, file, indent=4)


if __name__ == "__main__":
    main()