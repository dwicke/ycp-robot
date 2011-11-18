#!/usr/bin/python

# Copyright (C) 2011 Google Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License. You may obtain a copy of
# the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations under
# the License.

__author__ = 'damonkohler@google.com (Damon Kohler)'

import os

import base_test_case
import ros_properties
import resources
import roslib

SAMPLE_PACKAGE = 'sample_package'


class TestGenerateRosProperties(base_test_case.BaseTestCase):

    def test_get_package_version(self):
        self.assertEqual(42, ros_properties._get_package_version(
            'package', {'package': 'stack'}, {'stack': 42}))

    def test_generate_properties(self):
        rospack = roslib.packages.ROSPackages()
        properties = ros_properties.generate(rospack, SAMPLE_PACKAGE,
                                             {'compile': [], 'runtime': [], 'test': []})
        self.assertEqual('target/com.domain.sample.built_with_location-0.0.0.jar',
                         properties['ros.artifact.built'])
        expected = os.path.join(resources.get_resources_directory(), 'sample_stack',
                                'sample_package_dependency')
        self.assertEqual(expected, properties['ros.pkg.sample_package_dependency.dir'])