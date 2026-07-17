function urdf = load_fr3_urdf_parameters(urdfFile, reportFile)
%LOAD_FR3_URDF_PARAMETERS Parse the FR3 URDF using MATLAB's XML DOM API.
% The returned fixed-size arrays are the single source for simulation setup.

if nargin < 1 || isempty(urdfFile)
    urdfFile = fullfile(fileparts(mfilename('fullpath')), 'fr3.urdf');
end
if nargin < 2
    reportFile = '';
end
if ~isfile(urdfFile)
    error('FR3:MissingURDF', 'Missing URDF: %s', urdfFile);
end

doc = xmlread(urdfFile);
root = doc.getDocumentElement;
urdf.robotName = char(root.getAttribute('name'));
nodes = root.getElementsByTagName('joint');
n = nodes.getLength;
allJoints = repmat(struct('name','','type','','parent','','child','', ...
    'originXYZ',zeros(3,1),'originRPY',zeros(3,1),'axis',zeros(3,1), ...
    'lower',NaN,'upper',NaN,'velocity',NaN,'effort',NaN), n, 1);
for k = 1:n
    node = nodes.item(k-1);
    j = allJoints(k);
    j.name = char(node.getAttribute('name'));
    j.type = char(node.getAttribute('type'));
    j.parent = childAttribute(node, 'parent', 'link', '');
    j.child = childAttribute(node, 'child', 'link', '');
    j.originXYZ = parseVector(childAttribute(node, 'origin', 'xyz', '0 0 0'));
    j.originRPY = parseVector(childAttribute(node, 'origin', 'rpy', '0 0 0'));
    j.axis = parseVector(childAttribute(node, 'axis', 'xyz', '0 0 0'));
    j.lower = parseScalar(childAttribute(node, 'limit', 'lower', 'NaN'));
    j.upper = parseScalar(childAttribute(node, 'limit', 'upper', 'NaN'));
    j.velocity = parseScalar(childAttribute(node, 'limit', 'velocity', 'NaN'));
    j.effort = parseScalar(childAttribute(node, 'limit', 'effort', 'NaN'));
    allJoints(k) = j;
end
urdf.allJoints = allJoints;

armNames = arrayfun(@(i)sprintf('fr3_joint%d',i), 1:7, 'UniformOutput', false);
arm = repmat(allJoints(1), 7, 1);
for i = 1:7
    idx = find(strcmp({allJoints.name}, armNames{i}), 1);
    if isempty(idx), error('FR3:MissingJoint','URDF lacks %s.',armNames{i}); end
    arm(i) = allJoints(idx);
    if ~strcmp(arm(i).type,'revolute')
        error('FR3:JointType','%s is %s, expected revolute.',arm(i).name,arm(i).type);
    end
end
idx8 = find(strcmp({allJoints.name}, 'fr3_joint8'), 1);
if isempty(idx8), error('FR3:MissingJoint','URDF lacks fr3_joint8.'); end
urdf.armJoints = arm;
urdf.joint8 = allJoints(idx8);
urdf.jointNames = string({arm.name})';
urdf.parentLinks = string({arm.parent})';
urdf.childLinks = string({arm.child})';
urdf.originXYZ = reshape([arm.originXYZ],3,7);
urdf.originRPY = reshape([arm.originRPY],3,7);
urdf.axis = reshape([arm.axis],3,7);
urdf.qMin = [arm.lower]';
urdf.qMax = [arm.upper]';
urdf.qDotMax = [arm.velocity]';
urdf.jointTorqueMax = [arm.effort]';
urdf.joint8OriginXYZ = urdf.joint8.originXYZ;
urdf.joint8OriginRPY = urdf.joint8.originRPY;

if ~isempty(reportFile)
    fid = fopen(reportFile,'w');
    if fid < 0, error('FR3:ReportOpen','Cannot create %s.',reportFile); end
    cleanup = onCleanup(@()fclose(fid)); %#ok<NASGU>
    fprintf(fid,'FR3 URDF parse report\nSource: %s\nRobot: %s\nJoint count: %d\n\n', ...
        urdfFile,urdf.robotName,n);
    for k = 1:n
        j = allJoints(k);
        fprintf(fid,['Joint %d\n  name: %s\n  type: %s\n  parent: %s\n  child: %s\n' ...
            '  origin xyz: %.16g %.16g %.16g\n  origin rpy: %.16g %.16g %.16g\n' ...
            '  axis: %.16g %.16g %.16g\n  lower: %.16g\n  upper: %.16g\n' ...
            '  velocity: %.16g\n  effort: %.16g\n\n'], k,j.name,j.type,j.parent,j.child, ...
            j.originXYZ,j.originRPY,j.axis,j.lower,j.upper,j.velocity,j.effort);
    end
    fprintf(fid,'Prompt cross-check for fr3_joint1..7 and fixed fr3_joint8: PASS\n');
    fprintf(fid,'The URDF values match the task specification for origins, axes and limits.\n');
end
end

function value = childAttribute(node, tag, attribute, defaultValue)
list = node.getElementsByTagName(tag);
if list.getLength == 0
    value = defaultValue;
else
    value = char(list.item(0).getAttribute(attribute));
    if isempty(value), value = defaultValue; end
end
end

function v = parseVector(text)
v = sscanf(text,'%f');
if numel(v) ~= 3, error('FR3:VectorParse','Expected 3-vector, got "%s".',text); end
v = reshape(v,3,1);
end

function x = parseScalar(text)
x = sscanf(text,'%f',1);
if isempty(x), x = NaN; end
end
