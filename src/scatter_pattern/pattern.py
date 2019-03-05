from base_pattern import Pattern

import geometry_msgs.msg as gm
import numpy as np
import tf.transformations as tfs
import utilities


class PatternScatter(Pattern):

    _input_points = None

    def set_pattern_parameters(self, point_list):
        if not type(point_list) == list:
            utilities.output("ERROR: Point input is not a list of points")
            return False
        self._input_points = []
        for p in point_list:
            if not type(p) == list:
                utilities.output("ERROR: Single input point is not a list")
                return False
            self._input_points.append(p)
        self.set_parameterized(True)
        return True

    def generate_pattern(self):
        self._pattern = np.array(np.empty(len(self._input_points)), dtype=gm.Transform)
        i = 0
        for p in self._input_points:
            t = gm.Transform()
            if len(p) == 3:
                # it's just a coordinate
                t.translation.x = p[0]
                t.translation.y = p[1]
                t.translation.z = p[2]
                t.rotation.w = 1.0
            elif len(p) == 6:
                # it's a transform
                t.translation.x = p[0]
                t.translation.y = p[1]
                t.translation.z = p[2]
                q = tfs.quaternion_from_euler(p[3],p[4],p[5],axes='sxyz')
                t.rotation.x = q[0]
                t.rotation.y = q[1]
                t.rotation.z = q[2]
                t.rotation.w = q[3]
            else:
                utilities.output("ERROR: Incorrect point length (%s), aborting pattern generation" % len(p))
                return False
            self._pattern[i] = t
            i += 1
        self._pattern_org_copy = np.copy(self._pattern)
        self.finish_generation()
        return True


def main():
    print('I\'m a linear plugin')


if __name__ == '__main__':
    main()
