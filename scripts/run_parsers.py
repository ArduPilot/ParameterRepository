#!/usr/bin/env python3

import argparse
from typing import Tuple
import git
import glob
import os
import re
import shutil
import subprocess
import tempfile
from pathlib import Path

from packaging import version
from datetime import datetime, timezone

class Groundskeeper:
    repository_url = 'https://github.com/ArduPilot/ardupilot'
    repository_name = repository_url.split('/')[-1]
    tag_regex = re.compile(r'(?P<name>[A-z]+(\d+)?)(-(?P<major>\d+)\.(?P<minor>\d+)\.(?P<patch>\d+)?|-(?P<beta>beta))$')
    temp_folder = tempfile.mkdtemp()

    valid_name_map = {
        'AntennaTracker': 'Tracker',
        'AP_Periph': 'AP_Periph',
        'APMrover2': 'Rover',
        'ArduCopter': 'Copter',
        'ArduPlane': 'Plane',
        'ArduSub': 'Sub',
        'Blimp': 'Blimp',
        'Copter': 'Copter',
        'Plane': 'Plane',
        'Rover': 'Rover',
        'Sub': 'Sub',
        'Tracker': 'Tracker'
    }

    def __init__(self):
        self.repository_path = os.path.join(self.temp_folder, self.repository_name)
        print(f"""Using:
    Temporary folder: {self.temp_folder}
    Repository: {self.repository_url}
""")

    def get_version_for_tag(self, tag) -> Tuple[int, int]:
        if tag['major'] > 0:
            return tag['major'], tag['minor']
        else:
            # fetch number from version.h in the vehicle folder in the given tag
            self.repository.git.checkout(tag["tag"], force=True)

            version = self.get_version_from_source(tag['name'])
            if version is None:  # Try with an alternative path
                version = self.get_version_from_source(tag['vehicle_type'])
            if version is None:  # No luck
                print(f'Failed to get version for {tag["vehicle_type"]}')
                return (0, 0)
            return version[:2]  # major, minor

    def get_version_from_source(self, vehicle_type: str) -> Tuple[int, int, int] | None:
        # try to read <vehicle_type>/version.h
        repo_path = Path(self.repository_path)
        prefix = self.get_vehicle_prefix(vehicle_type)
        version_file = repo_path / f'{prefix}{vehicle_type}' / 'version.h'

        if not version_file.exists():
            return

        content = version_file.read_text()
        return tuple(
            int(re.search(rf'#define\s+FW_{v}\s+(\d+)', content).group(1))
            for v in ('MAJOR', 'MINOR', 'PATCH')
        )

    def clone_repository(self):
        print(f'Starting cloning to: {self.repository_path}')
        #self.repository_path = '/tmp/tmpetcgh7ni/ardupilot'
        #return git.Repo(self.repository_path)
        return git.Repo.clone_from(self.repository_url, self.repository_path)

    def tag_latest_versions(self):
        ''' Creates a VehicleType-Major.Minor.Patch tag from master, for each vehicle type.

        The version is extracted from the version.h file in the vehicle's source folder.
        '''
        for vehicle_type in set(self.valid_name_map.values()):
            if not (version := self.get_version_from_source(vehicle_type)):
                continue
            major, minor, patch = version
            try:
                self.repository.create_tag(f'{vehicle_type}-{major}.{minor}.{patch}')
            except Exception:
                pass  # If the tag already exists, we are happy

    @staticmethod
    def get_last_ground_change(repository: git.Repo|None = None):
        if repository is None:
            # Determine last ground change of the repo of this file
            source = git.Git(Path(__file__).parent.parent)
            # Ignore irrelevant files, and extract date of latest change in an unambiguous format
            last_relevant_commit_date = source.log('-1', '--format=%cI', '--', ':^.github', ':^README.md', ':^scripts')
            return datetime.fromisoformat(last_relevant_commit_date)

        # Determine last ground change of some other repo
        last_commit_date = repository.head.commit.committed_date
        return datetime.fromtimestamp(last_commit_date).astimezone(timezone.utc)

    @staticmethod
    def get_vehicle_prefix(vehicle_type: str):
        if vehicle_type == 'AP_Periph':
            return 'Tools/'
        if vehicle_type == 'Tracker':
            return 'Antenna'
        if vehicle_type in ('Copter', 'Plane', 'Sub'):
            return 'Ardu'
        return ''  # No prefix, or unknown vehicle type

    def run(self, since: datetime | None = None):
        self.repository = self.clone_repository()
        self.tag_latest_versions()  # Create temporary/local tags, to include development versions in metadata generation
        tag_names = [tag.path[len('refs/tags/'):] for tag in self.repository.tags]
        last_ground_change = since if since is not None else self.get_last_ground_change()

        # Prepare for MAVLink parsing - always use the latest script (since it might cover new dialects or messages)
        shutil.copy(f'{self.repository_path}/Tools/scripts/mavlink_parse.py', self.temp_folder)

        # Get only valid tag names
        tags = []
        for tag in tag_names:
            tag_data = self.tag_regex.search(tag)
            if not tag_data:
                continue  # Not a recognisable tag --> ignore
            matches = tag_data.groupdict()
            if not (vehicle_type := self.valid_name_map.get(matches['name'])):
                continue  # Not a vehicle tag --> ignore
            major, minor, patch = (int(matches[field] or 0) for field in ('major', 'minor', 'patch'))
            tags.append({
                'tag': tag_data[0],
                'reference': tag,
                'name': matches['name'],
                'vehicle_type': vehicle_type,
                'major': major,
                'minor': minor,
                'patch': patch,
                'beta': matches['beta'],
            })

        # Sort the tags so they appear in sequence, grouped by vehicle type
        tags.sort(key=lambda tag: tuple(tag[field] for field in ('vehicle_type', 'major', 'minor', 'patch')))

        # Get only the newest patch of each version
        old_versions = []
        previous_tag = None
        for tag in tags:
            # Beta releases are unique, we don't need to compare them
            if tag['beta']:
                continue
            if previous_tag:
                patch_update = (
                    tag['vehicle_type'] == previous_tag['vehicle_type']
                    and tag['major'] == previous_tag['major']
                    and tag['minor'] == previous_tag['minor']
                )
                if patch_update:
                    print(f' => {tag["tag"]}', end='')
            if not previous_tag or not patch_update:
                print('\n', tag['tag'], end='')
                previous_tag = tag
                continue
            # Tags are sorted, so this is a patch update
            old_versions.append(previous_tag['tag'])
            previous_tag = tag
        print()

        tags = [tag for tag in tags if tag['tag'] not in old_versions]

        # Generate parameters for all tags
        for tag in tags:
            tag_name, vehicle_type, tag_reference = (
                tag[field] for field in ('tag', 'vehicle_type', 'reference')
            )
            tag_major_version, tag_minor_version = self.get_version_for_tag(tag)
            folder_name = f'{vehicle_type}-{tag_major_version}.{tag_minor_version}'

            if not vehicle_type:
                continue

            print(f'Processing: {folder_name}..')
            # Old versions are not mantained and generation of the files is not fully supported
            if vehicle_type != "AP_Periph" and tag_major_version < 4:
                print("Ignoring old version")
                continue

            # Ignore local changes (they're automated anyway)
            self.repository.git.checkout(tag_reference, force=True)
            tag_date = self.get_last_ground_change(self.repository)
            if last_ground_change > tag_date:
                print(f"Version already generated for {tag_date}")
                continue

            # Run parameters parser
            try:
                subprocess.run([f'{self.repository_path}/Tools/autotest/param_metadata/param_parse.py', '--vehicle', vehicle_type], cwd=self.repository_path)
            except Exception as exception:
                print(exception)

            output = Path(__file__).parent.parent
            subprocess.run(['mkdir', '-p', output])
            dest = f"{output}/{folder_name}"
            subprocess.run(['mkdir', '-p', dest])
            for data in glob.glob(f'{dest}/*'):
                    os.remove(data)
            for data in glob.glob(f'{self.repository_path}/Parameter*'):
                shutil.copy2(data, dest)
                os.remove(data)
            for data in glob.glob(f'{self.repository_path}/apm.pdef.*'):
                shutil.copy2(data, dest)
                os.remove(data)

            if vehicle_type == 'AP_Periph':
                print('Skipping MAVLink parsing for AP_Periph firmware - unsupported.')
                continue

            # Run MAVLink messages parser
            vehicle = f'{self.get_vehicle_prefix(vehicle_type)}{vehicle_type}'
            # The parser expects to be run from its normal place in the repository
            script_folder = f'{self.repository_path}/Tools/scripts/'
            script_file = script_folder + 'mavlink_parse.py'
            # Stream groups were added in 4.1, and removed in 4.7, so only include them when relevant
            stream_groups = 'g' if (4, 0) < (tag_major_version, tag_minor_version) < (4, 7) else ''
            try:
                shutil.copy(f'{self.temp_folder}/mavlink_parse.py', script_file)
                subprocess.run(['python', script_file, f'-c{stream_groups}uq', '--header', 'ardupilot_wiki', '--format', 'rst',
                                '--filename', f'{dest}/MAVLinkMessages.rst', '--branch', folder_name, '--vehicle', vehicle],
                               cwd=script_folder)
            except Exception as exception:
                print(exception)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Generate parameter files for ArduPilot vehicle versions.'
    )
    parser.add_argument(
        '--since',
        type=lambda s: datetime.fromisoformat(s),
        default=None,
        help='Override the "latest" date to regenerate older versions. '
             'Versions with tags newer than this date will be regenerated. '
             'Use ISO 8601 format, e.g. 2020-01-01 or 2020-01-01T00:00:00+00:00'
    )
    args = parser.parse_args()

    G = Groundskeeper()
    G.run(since=args.since)
