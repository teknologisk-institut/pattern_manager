#!/usr/bin/env python

# Copyright 2019 Danish Technological Institute (DTI)

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#     http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Author: Mads Vainoe Baatrup

import rospy
import tf
import yaml

from pattern_manager import util
from pattern_manager import patterns
from pattern_manager import XForm
from pattern_manager import srv as pm_srv
from pattern_manager import msg as pm_msg
from std_srvs.srv import Trigger, TriggerResponse
from collections import OrderedDict
from visualization_msgs.msg import MarkerArray


class PatternManagerNode(object):

    def __init__(self):
        XForm(name='root', parent=None, ref_frame='world')

        rospy.Service('~get_transform', pm_srv.GetTransformParams, self.cb_get_transform)
        rospy.Service('~get_transform_id', pm_srv.GetTransformId, self.cb_get_transform_id)
        rospy.Service('~get_transform_ids', pm_srv.GetIds, self.cb_get_transform_ids)
        rospy.Service('~create_transform', pm_srv.CreateGroup, self.cb_create_transform)
        rospy.Service('~set_active', pm_srv.SetActive, self.cb_set_active)
        rospy.Service('~remove_transform', pm_srv.TransformId, self.cb_remove_transform)
        rospy.Service('~iterate', Trigger, self.cb_iterate)
        rospy.Service('~update_transform_variable', pm_srv.UpdateTransform, self.cb_update_tf_variable)
        rospy.Service('~get_current_transform_id', pm_srv.GetCurrentId, self.cb_get_current_tf_id)
        rospy.Service('~get_active_ids', pm_srv.GetIds, self.cb_get_active_ids)
        rospy.Service('~create_linear_pattern', pm_srv.CreateLinearPattern, self.cb_create_linear_pattern)
        rospy.Service('~create_rectangular_pattern', pm_srv.CreateRectangularPattern, self.cb_create_rectangular_pattern)
        rospy.Service('~create_circular_pattern', pm_srv.CreateCircularPattern, self.cb_create_circular_pattern)
        rospy.Service('~create_scatter_pattern', pm_srv.CreateScatterPattern, self.cb_create_scatter_pattern)
        rospy.Service('~set_iteration_order', pm_srv.SetIterationOrder, self.cb_set_iteration_order)
        rospy.Service('~set_transform_parent', pm_srv.SetParent, self.cb_set_transform_parent)
        rospy.Service('~save', pm_srv.Filename, self.cb_save)
        rospy.Service('~load', pm_srv.Filename, self.cb_load)

    @staticmethod
    def cb_get_transform_id(req):
        """
        This callback function retrieves an XForm ID from a supplied XForm name

        :param req: A request object containing the transform name
        :type req: GetTransformIdRequest
        :return: A response object containing the transform ID
        :rtype: GetTransformIdResponse
        """

        rospy.logdebug("Received request to retrieve transform id")
        resp = pm_srv.GetTransformIdResponse()

        try:
            for n in XForm.get_nodes():
                if n.name == req.name:
                    resp.id = id(n)
        except rospy.ROSException, e:
            rospy.logerr(e)

        return resp

    @staticmethod
    def cb_load(req):
        """
        This callback function loads a .yaml file into a dictionary to create an XForm tree from

        :param req: A request object containing the path of the .yaml file to load
        :type req: FileNameRequest
        :return: A response object containing a boolean whether the call was successful or not
        :rtype: FileNameResponse
        """

        rospy.logdebug("Received request to load tree")
        resp = pm_srv.FilenameResponse()

        try:
            with open(req.path, 'r') as file_:
                tree = yaml.load(file_)

            XForm.recursive_remove_node(id(XForm.root))
            XForm.from_dict(tree)

            file_.close()

            resp.success = True
        except rospy.ROSException, e:
            resp.success = False
            rospy.logerr(e)

        return resp

    @staticmethod
    def cb_save(req):
        """
        This callback function saves the XForm tree to a .yaml file representation

        :param req: A request object containing the filename of the .yaml file to save
        :type req: FileNameRequest
        :return: A response object containing a boolean whether the call was successful or not
        :rtype: FileNameResponse
        """

        rospy.logdebug("Received request to save tree")
        resp = pm_srv.FilenameResponse()

        try:
            with open(req.path, 'w') as file_:
                yaml.dump(XForm.to_dict(), file_)

            file_.close()

            resp.success = True
        except rospy.ROSException, e:
            resp.success = False
            rospy.logerr(e)

        return resp

    @staticmethod
    def cb_set_transform_parent(req):
        """
        This callback function sets the parent object of an XForm

        :param req: A request object containing the ID of the XForm and the ID of the parent
        :type req: SetParentRequest
        :return: A response object containing a boolean whether the call was successful or not
        :rtype: SetParentResponse
        """

        rospy.logdebug("Received request to move transform")
        resp = pm_srv.SetParentResponse()

        try:
            t = XForm.get_node(req.id)
            del t.parent.children[req.id]

            new_parent = XForm.get_node(req.parent_id)
            new_parent.add_node(t)

            t.ref_frame = new_parent.name
        except rospy.ROSException, e:
            resp.success = False
            rospy.logerr(e)

        return resp

    @staticmethod
    def cb_get_transform_ids(req):
        """
        This callback function retrieves the IDs of all currently existing XForm objects

        :param req: An empty request object
        :type req: GetIdsRequest
        :return: A response object containing a list of IDs
        :rtype: GetIdsResponse
        """

        rospy.logdebug("Received request to retrieve all transform ids")
        resp = pm_srv.GetIdsResponse()

        try:
            for n in XForm.get_nodes():
                resp.ids.append(id(n))
        except rospy.ROSException, e:
            rospy.logerr(e)

        return resp

    @staticmethod
    def cb_set_iteration_order(req):
        """
        This callback function sets the order of an XForms children

        :param req: A request object containing an XForm ID and a list of child IDs in a fixed order
        :type req: SetIterationOrderRequest
        :return: A response object containing a boolean whether the call was successful or not
        :rtype: SetIterationOrderResponse
        """

        rospy.logdebug("Received request to set iteration order")
        resp = pm_srv.SetIterationOrderResponse()

        try:
            t = XForm.get_node(req.id)

            ordered = OrderedDict()
            for k in req.order:
                ordered[k] = t.children[k]

            t.children = ordered

            resp.success = True
        except rospy.ROSException, e:
            resp.success = False
            rospy.logerr(e)

        return resp

    @staticmethod
    def cb_create_linear_pattern(req):
        """
        This callback function generates an XForm pattern in a linear shape

        :param req: A request object containing an XForm ID and a list of child IDs in a fixed order
        :type req: SetIterationOrderRequest
        :return: A response object containing a boolean whether the call was successful or not
        :rtype: SetIterationOrderResponse
        """

        rospy.logdebug("Received request to create a linear pattern")
        resp = pm_srv.CreateLinearPatternResponse()

        try:
            args = [req.num_points, req.step_size, req.length]

            t = XForm(XForm.get_node(req.parent.parent_id), name=req.parent.name)
            t.translation = req.parent.translation
            t.rotation = req.parent.rotation

            patterns.create_pattern(patterns.Pattern.linear, t, args)

            rospy.logout("Linear pattern successfully created!")
            resp.success = True
        except rospy.ROSException, e:
            rospy.logerr(e)
            resp.success = False

        return resp

    @staticmethod
    def cb_create_rectangular_pattern(req):
        rospy.logdebug("Received request to create a rectangular pattern")
        resp = pm_srv.CreateRectangularPatternResponse()

        try:
            args = [req.num_points, req.step_sizes, req.lengths]

            t = XForm(XForm.get_node(req.parent.parent_id), name=req.parent.name)
            t.translation = req.parent.translation
            t.rotation = req.parent.rotation

            patterns.create_pattern(patterns.Pattern.rectangular, t, args)

            rospy.logout("Rectangular pattern successfully created!")
            resp.success = True
        except rospy.ROSException, e:
            rospy.logerr(e)
            resp.success = False

        return resp

    @staticmethod
    def cb_create_scatter_pattern(req):
        rospy.logdebug("Received request to create a rectangular pattern")
        resp = pm_srv.CreateScatterPatternResponse()

        try:
            points = []
            for p in req.points:
                points.append(p.point)

            args = [points]

            t = XForm(XForm.get_node(req.parent.parent_id), name=req.parent.name)
            t.translation = req.parent.translation
            t.rotation = req.parent.rotation

            patterns.create_pattern(patterns.Pattern.scatter, t, args)

            rospy.logout("Scatter pattern successfully created!")
            resp.success = True
        except rospy.ROSException, e:
            rospy.logerr(e)
            resp.success = False

        return resp

    @staticmethod
    def cb_create_circular_pattern(req):
        rospy.logdebug("Received request to create a circular pattern")
        resp = pm_srv.CreateCircularPatternResponse()

        try:
            args = [req.num_points, req.radius, req.tan_rot, req.cw]

            t = XForm(XForm.get_node(req.parent.parent_id), name=req.parent.name)
            t.translation = req.parent.translation
            t.rotation = req.parent.rotation

            patterns.create_pattern(patterns.Pattern.circular, t, args)

            rospy.logout("Circular pattern successfully created!")
            resp.success = True
        except rospy.ROSException, e:
            rospy.logerr(e)
            resp.success = False

        return resp

    @staticmethod
    def cb_get_active_ids(self):
        rospy.logdebug("Received request to retrieve active transform ids")
        resp = pm_srv.GetIdsResponse()

        for n in XForm.get_active_nodes():
            resp.ids.append(id(n))

        return resp

    @staticmethod
    def cb_get_current_tf_id(req):
        rospy.logdebug("Received request to retrieve current transform id")
        resp = pm_srv.GetCurrentIdResponse()

        id_ = id(XForm.get_current_node())

        if id_:
            resp.id = id_
            resp.success = True
        else:
            resp.success = False

        return resp

    @staticmethod
    def cb_iterate(req):
        rospy.logdebug('Received request to iterate')
        resp = TriggerResponse()

        if XForm.get_current_node():
            XForm.iterate()

            resp.success = True
        else:
            resp.success = False

        return resp

    @staticmethod
    def cb_update_tf_variable(req):
        rospy.logdebug('Received request to change value of attribute for transform %s' % req.id)
        resp = pm_srv.UpdateTransformResponse()

        try:
            n = XForm.get_node(req.id)

            n.name = req.name
            n.ref_frame = req.ref_frame
            n.translation = req.translation
            n.rotation = req.rotation

            rospy.logout("Transform %s variable succesfully changed" % req.id)
            resp.success = True
        except rospy.ROSException, e:
            rospy.logerr(e)
            resp.success = False

        return resp

    @staticmethod
    def cb_get_transform(req):
        rospy.logdebug("Received request to retrieve all transforms")
        resp = pm_srv.GetTransformParamsResponse()

        try:
            t = XForm.get_node(req.id)

            t_params = pm_msg.Params()
            t_params.name = t.name
            t_params.parent_id = id(t)
            t_params.id = id(t)
            t_params.ref_frame = t.ref_frame
            t_params.active = t.active
            t_params.translation = t.translation
            t_params.rotation = t.rotation
            t_params.number = t.number

            if t.parent:
                t_params.parent_id = id(t.parent)

            resp.params = t_params
        except rospy.ROSException, e:
            rospy.logerr(e)

        return resp

    @staticmethod
    def cb_create_transform(req):
        rospy.logdebug("Received request to create transform")
        resp = pm_srv.CreateGroupResponse()

        if not req.group_name:
            rospy.logwarn("Transform must have a name")
            resp.success = False

            return resp

        try:
            XForm(XForm.get_node(req.parent_id), name=req.group_name, ref_frame=req.ref_frame)
            rospy.logout("Transform {} successfully created!".format(req.group_name))
            resp.success = True
        except rospy.ROSException, e:
            rospy.logerr(e)
            resp.success = False

        return resp

    @staticmethod
    def cb_set_active(req):
        rospy.logdebug('Received request to set active transform %s', req.id)
        resp = pm_srv.SetActiveResponse()

        try:
            XForm.get_node(req.id).set_active(req.active)

            rospy.logout("Transform {} successfully (de)activated!".format(req.id))
            resp.success = True
        except rospy.ROSException, e:
            rospy.logerr(e)
            resp.success = False

        return resp

    @staticmethod
    def cb_remove_transform(req):
        rospy.logdebug('Received request to remove transform %s', req.id)
        resp = pm_srv.TransformIdResponse()

        try:
            if id(XForm.root) == req.id:
                rospy.logwarn("Removing root is not allowed. Ignoring action")
                resp.success = False

                return resp

            XForm.recursive_remove_node(req.id)
            rospy.logout("Transform {} successfully removed!".format(req.id))
            resp.success = True
        except rospy.ROSException, e:
            rospy.logerr(e)
            resp.success = False

        return resp


if __name__ == "__main__":
    rospy.init_node("pattern_manager", log_level=rospy.DEBUG)

    pmn = PatternManagerNode()
    rospy.loginfo("Pattern manager node started")

    br = tf.TransformBroadcaster()
    pub = rospy.Publisher("visualization_marker_array", MarkerArray, queue_size=10)

    r = rospy.Rate(5)

    while not rospy.is_shutdown():
        nodes = XForm.get_nodes()
        actv_nodes = XForm.get_active_nodes()

        util.broadcast_transforms(br, nodes)
        util.publish_markers(pub, actv_nodes)

        r.sleep()
