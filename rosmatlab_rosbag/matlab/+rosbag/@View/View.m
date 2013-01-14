classdef View < handle

    properties (SetAccess = private, Hidden, Transient)
        handle = 0
    end

    properties (SetAccess = private, Dependent)
        Time
        Topic
        DataType
        MD5Sum
        MessageDefinition
        ConnectionHeader
        IsLatching

        Size
        Queries
        Connections
        BeginTime
        EndTime
    end

    properties
        UserData
    end

    events
        Callback
    end

    methods
        function obj = View(varargin)
            obj.handle = internal(obj, 'create', varargin{:});
        end

        function delete(obj)
            internal(obj, 'delete');
            obj.handle = 0;
        end

        function result = get.Size(obj)
            result = internal(obj, 'getSize');
        end

        function addQuery(obj, bag, varargin)
            internal(obj, 'addQuery', bag, varargin{:});
        end

        function reset(obj)
            internal(obj, 'reset');
        end

        function result = start(obj)
            result = internal(obj, 'start');
        end

        function result = valid(obj)
            result = internal(obj, 'valid');
        end

        function result = eof(obj)
            result = internal(obj, 'eof');
        end

        function [message, topic, datatype, varargout] = next(obj, varargin)
            nargoutchk(0, 5);
            [message, topic, datatype, varargout{1:nargout-3}] = internal(obj, 'next', varargin{:});
            if (~isempty(message)); notify(obj, 'Callback', ros.MessageEvent(message, topic, datatype, obj.MD5Sum)); end
        end

        function varargout = get(obj, varargin)
            nargoutchk(0, 5);
            [varargout{1:nargout}] = internal(obj, 'get', varargin{:});
        end

        function data = data(obj, varargin)
            data = internal(obj, 'data', varargin{:});
        end

        function result = get.Time(obj)
            result = internal(obj, 'getTime');
        end

        function result = get.Topic(obj)
            result = internal(obj, 'getTopic');
        end

        function result = get.DataType(obj)
            result = internal(obj, 'getDataType');
        end

        function result = get.MD5Sum(obj)
            result = internal(obj, 'getMD5Sum');
        end

        function result = get.MessageDefinition(obj)
            result = internal(obj, 'getMessageDefinition');
        end

        function result = get.ConnectionHeader(obj)
            result = internal(obj, 'getConnectionHeader');
        end

        function result = get.IsLatching(obj)
            result = internal(obj, 'isLatching');
        end

        function result = get.Queries(obj)
            result = internal(obj, 'getQueries');
        end

        function result = get.Connections(obj)
            result = internal(obj, 'getConnections');
        end

        function result = get.BeginTime(obj)
            result = internal(obj, 'getBeginTime');
        end

        function result = get.EndTime(obj)
            result = internal(obj, 'getEndTime');
        end

    end
end
