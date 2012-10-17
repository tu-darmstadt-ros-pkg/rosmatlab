classdef Subscriber < handle
    %UNTITLED3 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (SetAccess = private, Hidden = true)
        handle = 0
    end
    
    properties (Dependent = true, SetAccess = private)
        Topic
        NumPublishers
    end
    
    events
        Callback
    end
    
    methods
        function obj = Subscriber(varargin)
            obj.handle = ros_subscriber(obj, 'create', varargin{:});
        end

        function delete(obj)
            ros_subscriber(obj, 'delete');
            obj.handle = 0;
        end
        
        function result = subscribe(obj, topic, datatype, varargin)
            result = ros_subscriber(obj, 'subscribe', topic, datatype, varargin{:});
        end

        function result = poll(obj, varargin)
            result = ros_subscriber(obj, 'poll', varargin{:});
        end
        
        function result = get.Topic(obj)
            result = ros_subscriber(obj, 'topic');
        end
        
        function result = get.NumPublishers(obj)
            result = ros_subscriber(obj, 'numpublishers');
        end
    end
end
