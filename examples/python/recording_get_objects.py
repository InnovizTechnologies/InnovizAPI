# Copyright 2021 Innoviz Technologies
#
# Licensed under the Innoviz Open Dataset License Agreement (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://github.com/InnovizTechnologies/InnovizAPI/blob/main/LICENSE.md
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Innoviz API Python get frame object example #

from innopy.api import FileReader, FrameDataAttributes, GrabType


recording_path = '../example_recordings/pc_plus.dbpt'
attr = [FrameDataAttributes(GrabType.GRAB_TYPE_DETECTIONS_SI)]

obj_dic = {
      0: "Passenger car",
      1: "Truck",
      2: "Motorcycle",
      3: "Bicycle",
      4: "Pedestrian",
      5: "Animal",
      6: "Hazard",
      7: "Unknown",
      8: "Over drivable",
      9: "Under drivable"
}


fr = FileReader(recording_path)
print("number of frames: ", fr.num_of_frames)
for i in range(fr.num_of_frames):
    res = fr.get_frame(i, attr)
    if res.success is True:
        obj = res.results['GrabType.GRAB_TYPE_DETECTIONS_SI']
        for j in range(len(obj)):

            # no more objects in current frame
            if obj[j]['summary']['id'] == 0:
                break

            # determine object's class type by checking existence probability
            for k in range(len(obj[j]['classification'])):
                if obj[j]['classification'][k][0] != 0:
                    index = k
                    break

            # print object details
            print(
                    "frame number:", i,
                    "object number in frame:", j,
                    "object id:", obj[j]['summary']['id'],
                    "classification: ", obj_dic[index],
                    "existance probability:", obj[j]['existence']['existenceProbability'],
                    "position:",
                    "x", obj[j]['position']['x'],
                    "y", obj[j]['position']['y'],
                    "z", obj[j]['position']['z']
            )

            index = None
print("End of recording")