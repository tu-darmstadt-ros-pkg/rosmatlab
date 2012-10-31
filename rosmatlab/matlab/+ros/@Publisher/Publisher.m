classdef Publisher < handle

    properties (SetAccess = private, Hidden = true)
        handle = 0
    end

    properties (Dependent = true, SetAccess = private)
        Topic
        NumSubscribers
        Latched
    end
    
    properties
        UserData
    end

    methods
        function obj = Publisher(varargin)
            obj.handle = mex(obj, 'create', varargin{:});
        end

        function delete(obj)
            mex(obj, 'delete');
            obj.handle = 0;
        end

        function result = advertise(obj, topic, datatype, varargin)
            result = mex(obj, 'advertise', topic, datatype, varargin{:});
        end

        function publish(obj, varargin)
            mex(obj, 'publish', varargin{:});
        end

        function result = get.Topic(obj)
            result = mex(obj, 'getTopic');
        end

        function result = get.NumSubscribers(obj)
            result = mex(obj, 'getNumSubscribers');
        end

        function result = get.Latched(obj)
            result = mex(obj, 'isLatched');
        end
    end
end
