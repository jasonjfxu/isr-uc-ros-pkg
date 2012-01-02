; Auto-generated. Do not edit!


(cl:in-package auction_srvs-srv)


;//! \htmlinclude AuctioneerService-request.msg.html

(cl:defclass <AuctioneerService-request> (roslisp-msg-protocol:ros-message)
  ((seller_node
    :reader seller_node
    :initarg :seller_node
    :type cl:string
    :initform "")
   (auction_data
    :reader auction_data
    :initarg :auction_data
    :type auction_msgs-msg:Auction
    :initform (cl:make-instance 'auction_msgs-msg:Auction)))
)

(cl:defclass AuctioneerService-request (<AuctioneerService-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AuctioneerService-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AuctioneerService-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name auction_srvs-srv:<AuctioneerService-request> is deprecated: use auction_srvs-srv:AuctioneerService-request instead.")))

(cl:ensure-generic-function 'seller_node-val :lambda-list '(m))
(cl:defmethod seller_node-val ((m <AuctioneerService-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader auction_srvs-srv:seller_node-val is deprecated.  Use auction_srvs-srv:seller_node instead.")
  (seller_node m))

(cl:ensure-generic-function 'auction_data-val :lambda-list '(m))
(cl:defmethod auction_data-val ((m <AuctioneerService-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader auction_srvs-srv:auction_data-val is deprecated.  Use auction_srvs-srv:auction_data instead.")
  (auction_data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AuctioneerService-request>) ostream)
  "Serializes a message object of type '<AuctioneerService-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'seller_node))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'seller_node))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'auction_data) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AuctioneerService-request>) istream)
  "Deserializes a message object of type '<AuctioneerService-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'seller_node) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'seller_node) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'auction_data) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AuctioneerService-request>)))
  "Returns string type for a service object of type '<AuctioneerService-request>"
  "auction_srvs/AuctioneerServiceRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AuctioneerService-request)))
  "Returns string type for a service object of type 'AuctioneerService-request"
  "auction_srvs/AuctioneerServiceRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AuctioneerService-request>)))
  "Returns md5sum for a message object of type '<AuctioneerService-request>"
  "dd49c058f2c13c485fb938ba1ba60499")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AuctioneerService-request)))
  "Returns md5sum for a message object of type 'AuctioneerService-request"
  "dd49c058f2c13c485fb938ba1ba60499")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AuctioneerService-request>)))
  "Returns full string definition for message of type '<AuctioneerService-request>"
  (cl:format cl:nil "~%~%~%string seller_node~%auction_msgs/Auction auction_data~%~%~%================================================================================~%MSG: auction_msgs/Auction~%Header header~%string command~%string task_type_name~%string subject~%string metrics~%duration length~%geometry_msgs/Point task_location~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AuctioneerService-request)))
  "Returns full string definition for message of type 'AuctioneerService-request"
  (cl:format cl:nil "~%~%~%string seller_node~%auction_msgs/Auction auction_data~%~%~%================================================================================~%MSG: auction_msgs/Auction~%Header header~%string command~%string task_type_name~%string subject~%string metrics~%duration length~%geometry_msgs/Point task_location~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AuctioneerService-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'seller_node))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'auction_data))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AuctioneerService-request>))
  "Converts a ROS message object to a list"
  (cl:list 'AuctioneerService-request
    (cl:cons ':seller_node (seller_node msg))
    (cl:cons ':auction_data (auction_data msg))
))
;//! \htmlinclude AuctioneerService-response.msg.html

(cl:defclass <AuctioneerService-response> (roslisp-msg-protocol:ros-message)
  ((response_info
    :reader response_info
    :initarg :response_info
    :type cl:string
    :initform "")
   (winner_id
    :reader winner_id
    :initarg :winner_id
    :type cl:string
    :initform "")
   (winner_cost
    :reader winner_cost
    :initarg :winner_cost
    :type cl:float
    :initform 0.0))
)

(cl:defclass AuctioneerService-response (<AuctioneerService-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AuctioneerService-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AuctioneerService-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name auction_srvs-srv:<AuctioneerService-response> is deprecated: use auction_srvs-srv:AuctioneerService-response instead.")))

(cl:ensure-generic-function 'response_info-val :lambda-list '(m))
(cl:defmethod response_info-val ((m <AuctioneerService-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader auction_srvs-srv:response_info-val is deprecated.  Use auction_srvs-srv:response_info instead.")
  (response_info m))

(cl:ensure-generic-function 'winner_id-val :lambda-list '(m))
(cl:defmethod winner_id-val ((m <AuctioneerService-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader auction_srvs-srv:winner_id-val is deprecated.  Use auction_srvs-srv:winner_id instead.")
  (winner_id m))

(cl:ensure-generic-function 'winner_cost-val :lambda-list '(m))
(cl:defmethod winner_cost-val ((m <AuctioneerService-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader auction_srvs-srv:winner_cost-val is deprecated.  Use auction_srvs-srv:winner_cost instead.")
  (winner_cost m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AuctioneerService-response>) ostream)
  "Serializes a message object of type '<AuctioneerService-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'response_info))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'response_info))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'winner_id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'winner_id))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'winner_cost))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AuctioneerService-response>) istream)
  "Deserializes a message object of type '<AuctioneerService-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'response_info) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'response_info) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'winner_id) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'winner_id) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'winner_cost) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AuctioneerService-response>)))
  "Returns string type for a service object of type '<AuctioneerService-response>"
  "auction_srvs/AuctioneerServiceResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AuctioneerService-response)))
  "Returns string type for a service object of type 'AuctioneerService-response"
  "auction_srvs/AuctioneerServiceResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AuctioneerService-response>)))
  "Returns md5sum for a message object of type '<AuctioneerService-response>"
  "dd49c058f2c13c485fb938ba1ba60499")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AuctioneerService-response)))
  "Returns md5sum for a message object of type 'AuctioneerService-response"
  "dd49c058f2c13c485fb938ba1ba60499")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AuctioneerService-response>)))
  "Returns full string definition for message of type '<AuctioneerService-response>"
  (cl:format cl:nil "~%~%~%string response_info~%string winner_id~%float64 winner_cost~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AuctioneerService-response)))
  "Returns full string definition for message of type 'AuctioneerService-response"
  (cl:format cl:nil "~%~%~%string response_info~%string winner_id~%float64 winner_cost~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AuctioneerService-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'response_info))
     4 (cl:length (cl:slot-value msg 'winner_id))
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AuctioneerService-response>))
  "Converts a ROS message object to a list"
  (cl:list 'AuctioneerService-response
    (cl:cons ':response_info (response_info msg))
    (cl:cons ':winner_id (winner_id msg))
    (cl:cons ':winner_cost (winner_cost msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'AuctioneerService)))
  'AuctioneerService-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'AuctioneerService)))
  'AuctioneerService-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AuctioneerService)))
  "Returns string type for a service object of type '<AuctioneerService>"
  "auction_srvs/AuctioneerService")