classdef MessageEvent < event.EventData
    properties
        message
        topic
        datatype
        md5sum
    end
    
    methods
        function obj = MessageEvent(message, topic, datatype, md5sum)
            obj.message = message;
            obj.topic = topic;
            obj.datatype = datatype;
            obj.md5sum = md5sum;
        end
    end
end
