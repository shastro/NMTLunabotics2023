#!/usr/bin/env python3

"""Fix `installed.json` reproducibility."""

import json
import sys


def main():
    """Run the main program."""
    filepath = sys.argv[1]
    data = json.load(open(filepath))

    data['packages'][0]['tools'].sort(key=lambda x: x['name'] + x['version'])
    json.dump(data, open(filepath, 'w'), indent=2)


if __name__ == '__main__':
    main()
