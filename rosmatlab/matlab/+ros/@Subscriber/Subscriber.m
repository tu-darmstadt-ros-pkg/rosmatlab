classdef Subscriber < handle

    properties (SetAccess = private, Hidden = true)
        handle = 0
        poll_timer = []
    end

    properties (Dependent = true, SetAccess = private)
        Topic
        NumPublishers
    end

    properties
        PollPeriod = 0.01
    end

    events
        Callback
    end

    methods
        function obj = Subscriber(varargin)
            obj.handle = mex(obj, 'create', varargin{:});
            obj.poll_timer = timer('ExecutionMode', 'fixedDelay', 'ObjectVisibility', 'off', 'TimerFcn', @(~,~) obj.poll(0));
        end

        function delete(obj)
            obj.stop();
            mex(obj, 'delete');
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
            result = mex(obj, 'subscribe', topic, datatype, varargin{:});
        end

        function message = poll(obj, varargin)
            message = mex(obj, 'poll', varargin{:});
            if (~isempty(message)); notify(obj, 'Callback', ros.MessageEvent(message)); end
        end

        function result = getConnectionHeader(obj)
            result = mex(obj, 'getConnectionHeader');
        end

        function result = getReceiptTime(obj)
            result = mex(obj, 'getReceiptTime');
        end

        function result = get.Topic(obj)
            result = mex(obj, 'getTopic');
        end

        function result = get.NumPublishers(obj)
            result = mex(obj, 'getNumPublishers');
        end

        function set.PollPeriod(obj, period)
            obj.stop();
            obj.PollPeriod = period;
            obj.start();
        end
    end
end
