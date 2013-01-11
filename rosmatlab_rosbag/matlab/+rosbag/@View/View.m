classdef View < handle

    properties (SetAccess = private, Hidden, Transient)
        handle = 0
    end

    properties (SetAccess = private, Dependent)
        Size
        Time
        Topic
        DataType
        MD5Sum
        MessageDefinition
        ConnectionHeader
        IsLatching
        Queries
        Connections
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

        function result = eof(obj)
            result = internal(obj, 'eof');
        end

        function [message, topic, datatype, connectionHeader, receiptTime] = next(obj, varargin)
            nargoutchk(0, 5);
            message = internal(obj, 'next', varargin{:});
            topic = obj.Topic;
            datatype = obj.DataType;
            if (nargout >= 4); connectionHeader = obj.ConnectionHeader; end
            if (nargout >= 5); time = obj.Time; end

            if (~isempty(message)); notify(obj, 'Callback', ros.MessageEvent(message, topic, datatype, obj.MD5Sum)); end
        end

        function [message, topic, datatype, connectionHeader, receiptTime] = get(obj, varargin)
            nargoutchk(0, 5);
            message = internal(obj, 'get', varargin{:});
            topic = obj.Topic;
            datatype = obj.DataType;
            if (nargout >= 4); connectionHeader = obj.ConnectionHeader; end
            if (nargout >= 5); time = obj.Time; end
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

    end
end
