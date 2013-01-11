classdef Bag < handle

    properties (SetAccess = private, Hidden, Transient)
        handle = 0
    end

    properties (SetAccess = private, Dependent)
        FileName
        Mode
        MajorVersion
        MinorVersion
        Size
    end

    properties (Dependent)
        Compression
        ChunkThreshold
    end

    properties
        UserData
    end

    properties (Constant, Hidden)
        Write = uint8(1)
        Read = uint8(2)
        Append = uint8(4)

        Uncompressed = uint8(0)
        BZ2 = uint8(1)
    end

    methods
        function obj = Bag(varargin)
            obj.handle = internal(obj, 'create', varargin{:});
        end

        function delete(obj)
            internal(obj, 'delete');
            obj.handle = 0;
        end

        function open(obj, filename, varargin)
            internal(obj, 'open', filename, varargin{:});
        end

        function close(obj)
            internal(obj, 'close');
        end

        function write(obj, topic, datatype, data, varargin)
            internal(obj, 'write', topic, datatype, data, varargin{:})
        end

        % Get the filename of the bag
        function result = get.FileName(obj)
            result = internal(obj, 'getFileName');
        end

        % Get the mode the bag is in
        function result = get.Mode(obj)
            result = internal(obj, 'getMode');
        end

        % Get the major-version of the open bag file
        function result = get.MajorVersion(obj)
            result = internal(obj, 'getMajorVersion');
        end

        % Get the minor-version of the open bag file
        function result = get.MinorVersion(obj)
            result = internal(obj, 'getMinorVersion');
        end

        % Get the current size of the bag file (a lower bound)
        function result = get.Size(obj)
            result = internal(obj, 'getSize');
        end

        % Set the compression method to use for writing chunks
        function set.Compression(obj, varargin)
            internal(obj, 'setCompression', varargin{:});
        end

        % Get the compression method to use for writing chunks
        function result = get.Compression(obj)
            result = internal(obj, 'getCompression');
        end

        % Set the threshold for creating new chunks
        function set.ChunkThreshold(obj, varargin)
            internal(obj, 'setChunkThreshold', varargin{:});
        end

        % Get the threshold for creating new chunks
        function result = get.ChunkThreshold(obj)
            result = internal(obj, 'getChunkThreshold');
        end

    end
end
