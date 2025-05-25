function debugLog(fmt, varargin)
% debugLog Conditionally prints debug messages based on global suppression flag.
% If the appdata 'SuppressDebug' is true, messages are suppressed. Otherwise printed via fprintf.
% Usage: debugLog('Value: %d\n', x);
    if isappdata(0, 'SuppressDebug') && getappdata(0, 'SuppressDebug')
        return;
    end
    fprintf(fmt, varargin{:});
end