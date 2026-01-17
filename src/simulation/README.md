# simulation

You may run into issues with a missing terrain.dae file. The issue is that the `terrain.dae` file is stored in **Git LFS** (Large File Storage), but hasn't been downloaded. To fix this, do the following:

1. Install git-lfs if not already installed
```bash
sudo apt install git-lfs
```

2. Initialize git-lfs in your local repo
```bash
git lfs install
```

3. Pull the actual LFS files
```bash
git lfs pull
```