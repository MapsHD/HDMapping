#!/usr/bin/python3

import os
import subprocess

PROJECT_DIRECTORY = os.path.join(os.path.abspath(os.path.dirname(__file__)))

FILE_LOCATIONS = [os.path.join(PROJECT_DIRECTORY, 'core'),
                  os.path.join(PROJECT_DIRECTORY, 'core_hd_mapping'),
                  os.path.join(PROJECT_DIRECTORY, 'apps'),
                  os.path.join(PROJECT_DIRECTORY, 'pybind'),
                  os.path.join(PROJECT_DIRECTORY, 'shared')]

FILE_EXTENSIONS = ['.cpp',
                   '.hpp',
                   '.c',
                   '.h']


def get_files(input_paths: list[str], extensions: list[str]):
    files = []

    for input_path in input_paths:
        for (dirpath, dirnames, filenames) in os.walk(input_path):
            for filename in filenames:
                if filename.endswith(tuple(extensions)):
                    files.append(os.path.join(dirpath, filename))
    return files


def format_file(file_path: str):
    print(f'Formatting: {file_path}')

    command = ['clang-format',  '-i', file_path]
    return subprocess.run(command).returncode


def format_files(file_paths: list[str]):
    for file_path in file_paths:
        format_file(file_path)


def main():
    format_files(get_files(FILE_LOCATIONS, FILE_EXTENSIONS))


if __name__ == '__main__':
    main()
