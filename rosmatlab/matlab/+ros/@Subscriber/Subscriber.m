classdef Subscriber < handle

    properties (SetAccess = private, Hidden, Transient)
        handle = 0
        poll_timer = []
    end

    properties (SetAccess = private)
        Topic = ''
        DataType = ''
        MD5Sum = ''
    end

    properties (SetAccess = private, Dependent)
        NumPublishers
    end

    properties
        PollPeriod = 0.01
        UserData
    end

    events
        Callback
    end

    methods
        function obj = Subscriber(varargin)
            obj.handle = internal(obj, 'create', varargin{:});
            obj.poll_timer = timer('ExecutionMode', 'fixedDelay', 'ObjectVisibility', 'off', 'TimerFcn', @(~,~) obj.poll(0));

            obj.Topic    = internal(obj, 'getTopic');
            obj.DataType = internal(obj, 'getDataType');
            obj.MD5Sum   = internal(obj, 'getMD5Sum');
        end

        function delete(obj)
            obj.stop();
            delete(obj.poll_timer);
            internal(obj, 'delete');
            obj.handle = 0;
        end

        function start(obj)
            set(obj.poll_timer, 'Period', obj.PollPeriod);
            start(obj.poll_timer);
        end

        function stop(obj)
            stop(obj.poll_timer);
        end

        function result = subscribe(obj, topic, datatype, varargin)
            result = internal(obj, 'subscribe', topic, datatype, varargin{:});
        end

        function [message, varargout] = poll(obj, varargin)
            nargoutchk(0, 3);
            [message, varargout{1:nargout-1}] = internal(obj, 'poll', varargin{:});
            if (~isempty(message)); notify(obj, 'Callback', ros.MessageEvent(message, obj.Topic, obj.DataType, obj.MD5Sum)); end
        end

        function result = getConnectionHeader(obj)
            result = internal(obj, 'getConnectionHeader');
        end

        function result = getReceiptTime(obj)
            result = internal(obj, 'getReceiptTime');
        end

        function result = get.NumPublishers(obj)
            result = internal(obj, 'getNumPublishers');
        end

        function set.PollPeriod(obj, period)
            obj.stop();
            obj.PollPeriod = period;
            obj.start();
        end
    end
end
