#!/usr/bin/env python3

"""Library for generating container infrastructure."""

from typing import List, Dict, Any, Optional
import os
import json
import shutil
import tempfile

HAVE_NVIDIA = True

class Compose:
    """A Docker compose file."""

    def __init__(self, name: str) -> None:
        """Create a new empty compose file."""
        self._name = name
        self._containers: Dict[str, Container] = {}
        self._network: str = f'{name}_network'
        self._external_network = False

    def add(self, name: str, container: 'Container') -> 'Compose':
        """Add a new container to the compose file."""
        self._containers[name] = container
        return self

    def connect(self, name: str) -> 'Compose':
        """Specify the external network to connect to."""
        # this network pattern is bad and inconsistent, but works for
        # now
        self._network = name
        self._external_network = True
        return self

    def network_host(self, name: str) -> 'Compose':
        """Specify the network to host."""
        self._network = name
        return self

    def construct(self) -> Any:
        """Construct the file as a JSON object."""
        result: Any = {
            'services': {},
            'networks': {
                self._network: {
                    'name': self._network,
                    'external': self._external_network,
                },
            },
        }
        for name in self._containers.keys():
            result['services'][name] = self._containers[name].construct(name)
            result['services'][name]['networks'] = [self._network]
        return result

    def run(self) -> None:
        """Execute the compose file."""
        filename = f'{self._name}.yml'

        with open(filename, 'w') as f:
            f.write(json.dumps(self.construct()))

        rv = os.system(f'sudo docker compose -f {filename} up --build')
        os.system(f'sudo docker compose -f {filename} down')

        for container in self._containers.values():
            container.cleanup()

        if rv != 0:
            raise Exception(f'Compose definition {filename} failed')


class Container:
    """A single container definition."""

    def __init__(self, build: str, command: str) -> None:
        """Create a new container."""
        self._command = command

        self._tmpdir = tempfile.mkdtemp()
        self._build = f'{self._tmpdir}/build'
        shutil.copytree(build, self._build, symlinks=True)

        self._environment: Dict[str, str] = {}
        self._volumes: List[str] = []
        self._gpu = False

    def cleanup(self) -> None:
        """Clean up the container's temporary files."""
        print(f'Removing {self._tmpdir}')
        shutil.rmtree(self._tmpdir)

    def env(self, key: str, value: str) -> 'Container':
        """Add an environment variable to the container."""
        self._environment[key] = value
        return self

    def volume(self, host: str, inner: str) -> 'Container':
        """Add a volume to the container."""
        self._volumes.append(f'{host}:{inner}')
        return self

    def mirror(self, path: str) -> 'Container':
        """Mirrors a path between the host and the container."""
        return self.volume(path, path)

    def ro_volume(self, host: str, inner: str) -> 'Container':
        """Add a read-only volume to the container."""
        return self.volume(host, f'{inner}:ro')

    def ro_mirror(self, path: str) -> 'Container':
        """Mirorrs a path read-only between the host and the container."""
        return self.ro_volume(path, path)

    def graphics(self) -> 'Container':
        """Add graphical capabilities to the container."""
        self._gpu = True

        display = os.getenv('DISPLAY')
        if display is None:
            raise Exception('No display available')

        # We gotta do some magic to make X11 accept connections from
        # within a Docker container.
        os.system('xhost +local:root')

        return (self
                .env('DISPLAY', display)
                .mirror('/tmp/.X11-unix')
                .mirror('/dev/dri')
        )

    def add_line(self, line: str) -> 'Container':
        """Add a line to the end of the Dockerfile."""
        with open(f'{self._build}/Dockerfile', 'a') as f:
            f.write(f'{line}\n')
        return self

    def add_cmd(self, cmd: str) -> 'Container':
        """Add a command to the end of the Dockerfile."""
        return self.add_line(f'RUN {cmd}')

    def add_pkg(self, pkg: str) -> 'Container':
        """Add a package to the container."""
        return self.add_cmd(f'apt-get install -y {pkg}')

    def construct(self, hostname: str) -> Any:
        """Construct the container as a JSON object."""
        result = {
            'hostname': hostname,
            'build': self._build,
            'environment': self._environment,
            'command': self._command,
            'volumes': self._volumes,
        }

        if self._gpu and HAVE_NVIDIA:
            result['deploy'] = {
                'resources': {
                    'reservations': {
                        'devices': [
                            {
                                'driver': 'nvidia',
                                'count': 1,
                                'capabilities': [
                                    'gpu',
                                ],
                            },
                        ],
                    },
                },
            }

        return result


def roscmd(
        name: str,
        cmd: Optional[str] = None,
        pkg: Optional[str] = None,
        master_address: str = 'ros',
) -> Compose:
    """Build a ROS-specific Docker command."""
    container = (Container('./ros', cmd or name)
                 .env('ROS_MASTER_URI', f'http://{master_address}:11311')
                 .graphics())
    if pkg:
        container.add_pkg(pkg)
    return (Compose(name)
            .add(name, container)
            .connect('ros-docker-net'))
