classdef MessageEvent < event.EventData
    properties
        message
    end
    
    methods
        function obj = MessageEvent(message)
            obj.message = message;
        end
    end
end
