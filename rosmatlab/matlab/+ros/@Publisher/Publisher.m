classdef Publisher < handle

    properties (SetAccess = private, Hidden, Transient)
        handle = 0
    end

    properties (SetAccess = private)
        Topic = ''
        DataType = ''
        MD5Sum = ''
        Latched = false
    end

    properties (SetAccess = private, Dependent)
        NumSubscribers
    end

    properties
        UserData
    end

    methods
        function obj = Publisher(varargin)
            obj.handle = internal(obj, 'create', varargin{:});

            obj.Topic    = internal(obj, 'getTopic');
            obj.DataType = internal(obj, 'getDataType');
            obj.MD5Sum   = internal(obj, 'getMD5Sum');
            %obj.Latched  = internal(obj, 'isLatched'); % currently not implemented
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

        function result = get.NumSubscribers(obj)
            result = internal(obj, 'getNumSubscribers');
        end
    end
end
