# NeoVim

## visual mode(nvim)
* used to select text(like a clig and drag mode)
* v-> character wise selection
* V -> line-wise selection
* ctrl + v -> block selection

then: 
* y -> copy
* d -> delete
* leave v mode and just press p to paste
* in no mode, undo with u

# docker
* docker build -t my-neovim .
* docker run -it --rm -v ${PWD}:/workspace my-neovim
* docker compose run --rm nvim (runs a container )
* docker compose up (runs a service or background thing)
* export TERM=xterm-256color

# tmux
* split vertically: ctrl + b then %
* split horizontally: ctrl + b then "
* create new window: ctrl + b then c
* navaigate panes: ctrl + b then arrows
* detach session: ctrl + b then d
* attach session: tmux attach or tmux attach -t <session_name>
* to navigate windows ctrl + b then n or p(short for next and prev)(or window number insteas of n or p)
