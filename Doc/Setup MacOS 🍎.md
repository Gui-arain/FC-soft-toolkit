# Setup

Installations steps for all required packages for NuttX on a Nucleo H753. We setup OpenOCD, the GNU toolchain for cpp and GDB for debug.

To intall NuttX follow the `NuttX setup.md` guide

## Steps:

1. Install core dependencies (Homebrew 🍺)
2. Install `flock` (required by NuttX build)
3. Install ARM GNU Toolchain (GCC + G++ + GDB)
4. Install correct `kconfig-mconf` (critical ⚠️)
5. Configure NuttX and build
6. Flash with OpenOCD
7. Debug (OpenOCD + GDB)

---

## 📦 1. Install core dependencies (Homebrew)

```bash
brew update
brew install openocd cmake ninja python pkg-config gperf flex bison node picocom
```
Also installs `picocom` to later use the nuttshell

---

## 🔧 2. Install `flock` (required by NuttX build)

macOS does not provide `flock` by default.

```bash
brew tap discoteq/discoteq
brew install flock
```

Verify:

```bash
which flock
```

---

## ⚙️ 3. Install ARM GNU Toolchain (GCC + G++ + GDB)

Use xPack (reliable on macOS):

```bash
npm install -g xpm

mkdir -p ~/toolchains/xpacks && cd ~/toolchains/xpacks
xpm init -y
xpm install @xpack-dev-tools/arm-none-eabi-gcc@latest
```

Add to PATH:

```bash
echo 'export PATH="$HOME/toolchains/xpacks/xpacks/.bin:$PATH"' >> ~/.zshrc
source ~/.zshrc
```

Verify:

```bash
arm-none-eabi-gcc --version
arm-none-eabi-gdb --version
```

---

## 🧩 4. Install correct `kconfig-mconf` (critical ⚠️)

⚠️ Do **NOT** use Homebrew’s `kconfig-mconf` → incompatible with NuttX.

Build from NuttX tools repo:

```bash
mkdir -p ~/tools
cd ~/tools
git clone https://bitbucket.org/nuttx/tools.git nuttx-tools
cd nuttx-tools/kconfig-frontends

patch < ../kconfig-macos.diff -p 1

./configure --prefix=$HOME/tools/kconfig-frontends-install \
  --enable-mconf \
  --disable-gconf \
  --disable-qconf \
  --disable-nconf

make -j
make install
```

Add to PATH (before Homebrew):

```bash
echo 'export PATH="$HOME/tools/kconfig-frontends-install/bin:$PATH"' >> ~/.zshrc
source ~/.zshrc
```

Verify:

```bash
which kconfig-mconf
```

Expected:

```
~/tools/kconfig-frontends-install/bin/kconfig-mconf
```

---

## 📁 5. Configure NuttX and build

From your `nuttx` directory:

```bash
make distclean || true
./tools/configure.sh -l nucleo-h743zi:nsh
```

Now we can build
```bash
make -j
```

Outputs:

- `nuttx` (ELF)
- `nuttx.bin`
- `nuttx.hex`

---

## 🔌 6. Flash with OpenOCD

Find board config:

```bash
ls "$(brew --prefix openocd)/share/openocd/scripts/board" | grep nucleo
```

Example:

```bash
openocd -f board/st_nucleo_h743zi.cfg \
  -c "init; reset halt" \
  -c "program nuttx.bin 0x08000000 verify reset exit"
```

---

## 🐞 8. Debug (GDB + OpenOCD)

### Terminal 1:

```bash
openocd -f board/st_nucleo_h743zi.cfg
```

### Terminal 2:

```bash
arm-none-eabi-gdb nuttx
```

Inside GDB:

```gdb
target extended-remote localhost:3333
monitor reset halt
load
break nx_start
continue
```

---

## ⚠️ Common Issues

### ❌ `unknown option "imply"` (Kconfig errors)

➡️ Wrong `kconfig-mconf` → rebuild from NuttX tools

---

### ❌ `flock: command not found`

➡️ Install with:

```bash
brew install flock
```

---

### ❌ `libapps.a: No such file`

➡️ Caused by failed build (often missing `flock`)

---

### ❌ Wrong compiler used

Check:

```bash
which arm-none-eabi-gcc
```

---

## ✅ Final sanity check

```bash
which arm-none-eabi-gcc
which kconfig-mconf
which flock
which openocd
```

---

## 💡 Recommendation (for your FC project)

Create a reusable environment script:

```bash
# env.sh
export PATH="$HOME/tools/kconfig-frontends-install/bin:$PATH"
export PATH="$HOME/toolchains/xpacks/xpacks/.bin:$PATH"
```

Then:

```bash
source env.sh
```
