# Devcontainer Configuration Guide

This directory contains VS Code devcontainer configurations for the ForzaETH Race Stack development environment.

## Available Configurations

### 1. Default Configuration (`devcontainer.json`)

**Recommended for:** Most users including Linux, Windows (WSL2), and standard Docker Desktop setups.

**Cache Location:** Inside the workspace at `./cache/`

```
TAM/ (or race_stack/)
├── .devcontainer/
│   └── devcontainer.json  <- Default config
├── cache/                  <- Cache stored here
│   └── noetic/
│       ├── build/
│       ├── devel/
│       └── logs/
└── ...
```

**Setup:**
```bash
cd <race_stack folder>
mkdir -p cache/noetic/{build,devel,logs}
```

This is the **default** and **recommended** configuration that works for most development environments.

### 2. MacOS + LimaVM Configuration (`devcontainer.macos-lima.json`)

**Recommended for:** MacOS users with LimaVM or other setups where mounting directories outside the workspace is problematic.

**Cache Location:** One directory up from workspace at `../cache/`

```
parent_dir/
├── TAM/ (or race_stack/)
│   ├── .devcontainer/
│   │   └── devcontainer.macos-lima.json  <- MacOS+LimaVM config
│   └── ...
└── cache/                  <- Cache stored here
    └── noetic/
        ├── build/
        ├── devel/
        └── logs/
```

**Setup:**
1. Rename the configuration file:
   ```bash
   cd <race_stack folder>/.devcontainer
   mv devcontainer.json devcontainer.json.bak
   mv devcontainer.macos-lima.json devcontainer.json
   ```

2. Create the cache directory:
   ```bash
   cd <race_stack folder>
   mkdir -p ../cache/noetic/{build,devel,logs}
   ```

## How to Choose

- **Use the default** (`devcontainer.json`) unless you encounter volume mount issues
- **Use MacOS+LimaVM** (`devcontainer.macos-lima.json`) if:
  - You're using MacOS with LimaVM
  - You experience issues with mounting directories inside the workspace
  - You need the cache outside the workspace for performance or compatibility reasons

## Switching Between Configurations

To switch from default to MacOS+LimaVM:
```bash
cd .devcontainer
cp devcontainer.json devcontainer.default.json  # backup default
cp devcontainer.macos-lima.json devcontainer.json
```

To switch back to default:
```bash
cd .devcontainer
cp devcontainer.default.json devcontainer.json
```

## Technical Details

Both configurations:
- Use the same Docker images (`race_stack_sim_x86`, `race_stack_sim_arm`, `race_stack_nuc`, `race_stack_jet`)
- Mount the same container paths (`/home/${USER}/catkin_ws/build`, `devel`, `logs`)
- Have identical VS Code extensions and settings
- Run the same post-installation scripts

The **only difference** is the host path for cache mounts:
- Default: `${localWorkspaceFolder}/cache/noetic/*`
- MacOS+LimaVM: `${localWorkspaceFolder}/../cache/noetic/*`

## Troubleshooting

### Error: "Cannot create directory"
- Ensure you've created the cache directory structure before launching the devcontainer
- Verify the cache path matches your chosen configuration

### Permission Issues
- On Linux, ensure your user has write permissions to the cache directory
- Run `chmod -R 775 cache` if needed

### Volume Mount Errors on MacOS
- Switch to the MacOS+LimaVM configuration
- Ensure LimaVM has access to the parent directory of your workspace

For more information, see the main [Docker README](../.docker_utils/README.md) and [Installation Guide](../INSTALLATION.md).
