classdef Publisher < handle

    properties (SetAccess = private, Hidden, Transient)
        handle = 0
    end

    properties (SetAccess = private, Dependent)
        Topic
        NumSubscribers
        Latched
    end
    
    properties
        UserData
    end

    methods
        function obj = Publisher(varargin)
            obj.handle = internal(obj, 'create', varargin{:});
        end

        function delete(obj)
            internal(obj, 'delete');
            obj.handle = 0;
        end

        function result = advertise(obj, topic, datatype, varargin)
            result = internal(obj, 'advertise', topic, datatype, varargin{:});
        end

        function publish(obj, varargin)
            internal(obj, 'publish', varargin{:});
        end

        function result = get.Topic(obj)
            result = internal(obj, 'getTopic');
        end

        function result = get.NumSubscribers(obj)
            result = internal(obj, 'getNumSubscribers');
        end

        function result = get.Latched(obj)
            result = internal(obj, 'isLatched');
        end
    end
end
