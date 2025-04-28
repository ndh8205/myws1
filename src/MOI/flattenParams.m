function flat = flattenParams(nodeArray)
flat = [];
for n = 1:numel(nodeArray)
    flat = [flat, nodeArray(n)];
    if ~isempty(nodeArray(n).Subcomponents)
        flat = [flat, flattenParams([nodeArray(n).Subcomponents{:}])];
    end
end
end
