import sys
import json

def get_source(path):
    if path.endswith('.launch.py'):
        from launch.launch_description_sources import PythonLaunchDescriptionSource
        return PythonLaunchDescriptionSource(path)
    else:
        from launch.launch_description_sources import FrontendLaunchDescriptionSource
        return FrontendLaunchDescriptionSource(launch_file_path=path)

def show_args(path):
    from launch.actions import DeclareLaunchArgument
    from launch.substitutions import TextSubstitution
    source = get_source(path)
    desc = source.try_get_launch_description_without_context()
    args = []
    for entity in desc.entities:
        if isinstance(entity, DeclareLaunchArgument):
            default = ''
            if entity.default_value is not None:
                parts = []
                for sub in entity.default_value:
                    if isinstance(sub, TextSubstitution):
                        parts.append(sub.text)
                    else:
                        parts.append(f'<{type(sub).__name__}>')
                default = ''.join(parts)
            args.append({'name': entity.name, 'default': default})
    json.dump(args, sys.stdout)

def run(path, launch_args):
    from launch import LaunchService
    from launch.actions import IncludeLaunchDescription
    source = get_source(path)
    ls = LaunchService()
    ls.include_launch_description(
        IncludeLaunchDescription(source, launch_arguments=launch_args)
    )
    sys.exit(ls.run())

if __name__ == '__main__':
    cmd = sys.argv[1]
    path = sys.argv[2]
    if cmd == 'show-args':
        show_args(path)
    elif cmd == 'run':
        args = []
        for a in sys.argv[3:]:
            if ':=' in a:
                k, v = a.split(':=', 1)
                args.append((k, v))
        run(path, args)
