#!/usr/bin/env python3

import json


N_SERVICES = 2
RANGE_SERVICES = range(1, N_SERVICES + 1)

def main():
    volumes = {}
    for i in RANGE_SERVICES:
        for base in 'root-gradle', 'project-gradle', 'project-build':
            volumes[f'{base}-{i}'] = None

    services = {}
    for i in RANGE_SERVICES:
        service = {
            'container_name': f'c{i}',
            'build': '.',
            'ports': [f'{5900 + i}:5900'],
            'volumes': [
                './:/coordination_oru/',
                f'project-gradle-{i}:/coordination_oru/.gradle/',
                f'project-build-{i}:/coordination_oru/build/',
                f'root-gradle-{i}:/root/.gradle/',
            ],
            'environment': {
                'WORKER': f'c{i}',
            },
            'command': 'container/index.sh',
            'network_mode': 'bridge',  # for the Internet to work
            'shm_size': '4gb',  # for Chromium
        }
        services[f'oru-{i}'] = service

    config = {'volumes': volumes, 'services': services}

    with open('compose.yaml', 'w') as file:
        json.dump(config, file, indent=4)


if __name__ == "__main__":
    main()