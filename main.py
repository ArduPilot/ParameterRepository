import git
import glob
import os
import re
import shutil
import subprocess
import tempfile

from packaging import version

class Groundskeeper:
    repository_url = 'https://github.com/ArduPilot/ardupilot'
    repository_name = repository_url.split('/')[-1]
    tag_regex = re.compile(r'(?P<name>[A-z]+(\d+)?)-(?P<major>\d+)\.(?P<minor>\d+)\.(?P<patch>\d+)?')
    temp_folder = tempfile.mkdtemp()

    valid_name_map = {
        'APMrover2': 'Rover',
        'ArduCopter': 'Copter',
        'ArduPlane': 'Plane',
        'ArduSub': 'Sub',
        'Copter': 'Copter',
        'Plane': 'Plane',
        'Rover': 'Rover',
    }

    def __init__(self):
        self.repository_path = os.path.join(self.temp_folder, self.repository_name)
        print(f"""Using:
    Temporary folder: {self.temp_folder}
    Repository: {self.repository_url}
""")


    def clone_repository(self):
        print(f'Starting cloning to: {self.repository_path}')
        #self.repository_path = '/tmp/tmpetcgh7ni/ardupilot'
        #return git.Repo(self.repository_path)
        return git.Repo.clone_from(self.repository_url, self.repository_path)

    def run(self):
        repository = self.clone_repository()
        tag_names = [tag.path[len('refs/tags/'):] for tag in repository.tags]

        # Get only valid tag names
        tags = [
            {
                'tag': self.tag_regex.search(tag)[0],
                'reference': tag,
                'matches': {**self.tag_regex.search(tag).groupdict()}
            } for tag in tag_names if self.tag_regex.search(tag)
        ]
        tags = [tag for tag in tags if self.valid_name_map.get(tag['matches']['name'])]

        # Get only the newest patch version
        old_versions = []
        previous_tag = None
        for tag in tags:
            if previous_tag:
                print(f'{previous_tag["tag"]} => {tag["tag"]}')
            if not previous_tag or (
                    tag['matches']['name'] != previous_tag['matches']['name']
                    or tag['matches']['major'] != previous_tag['matches']['major']
                    or tag['matches']['minor'] != previous_tag['matches']['minor']
                ):
                previous_tag = tag
                continue
            if tag['matches']['patch'] > previous_tag['matches']['patch']:
                print(f'Remove {previous_tag["tag"]}')
                old_versions.append(previous_tag['tag'])
                previous_tag = tag
            else:
                print(f'Remove {tag["tag"]}')
                old_versions.append(tag['tag'])

        tags = [tag for tag in tags if tag['tag'] not in old_versions]

        # Generate parameters for all tags
        for tag in tags:
            tag_name = tag['tag']
            tag_simple_name = tag['matches']['name']
            vehicle_type = self.valid_name_map.get(tag_simple_name)
            tag_reference = tag['reference']

            if not vehicle_type:
                continue

            print(f'Going to: {tag_name}')
            repository.git.checkout(tag_reference)
            try:
                subprocess.run([f'{self.repository_path}/Tools/autotest/param_metadata/param_parse.py', '--vehicle', vehicle_type])
            except Exception as exception:
                print(exception)

            output = f'/tmp/output'
            subprocess.run(['mkdir', '-p', output])
            dest = f"{output}/{tag_simple_name}-{tag['matches']['major']}.{tag['matches']['minor']}"
            subprocess.run(['mkdir', '-p', dest])
            for data in glob.glob('Parameter*'):
                shutil.move(data, dest)
            for data in glob.glob('apm.pdef.*'):
                shutil.move(data, dest)



G = Groundskeeper()
G.run()