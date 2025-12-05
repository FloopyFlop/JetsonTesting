# Connecting

Run `ssh [USERNAME]@192.168.55.1`.
Then type the `[PASSWORD]`.

# Wifi

Turn wifi on with `nmcli r wifi on`
List networks with `nmcli d wifi list`

Connect Log:
```
> nmcli connection show
NAME                UUID                                  TYPE      DEVICE 
l4tbr0              fcdc0214-0eef-43ca-8449-f69611971f98  bridge    l4tbr0 
ORBI65-Guest        c49eee42-1127-4741-8861-e40fe66e24e7  wifi      --     
Wired connection 1  155fb07a-56d8-37a5-821f-a148bd786b60  ethernet  --     
Wired connection 2  aa56aabe-7e57-3cb1-a825-8fee07545a77  ethernet  --     
> sudo nmcli connection add type wifi ifname wlP9p1s0 con-name eduroam ssid eduroam -- wifi-sec.key-mgmt wpa-eap 802-1x.eap peap 802-1x.phase2-auth mschapv2 802-1x.identity "[EMAIL]" 802-1x.anonymous "@cornell.edu" 802-1x.system-ca-certs no
Connection 'eduroam' (c21faf3b-c00c-4199-a442-2611654e92ee) successfully added.
> sudo nmcli connection modify eduroam 802-1x.password "[PASSWORD]"
> sudo nmcli connection up eduroam
```

# UV

Install uv with `curl -LsSf https://astral.sh/uv/install.sh | sh`

# Copying in files

When copying and overwriting use `rsync -avz --delete --ignore-times [DEST_FOLDER] magpie@192.168.55.1:~/abm-sync` or replace `abm-sync` with the dest folder, etc.
Then enter the password.

#TODO, figure out ros2 and get Mav DataHose running