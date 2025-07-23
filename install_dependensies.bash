#!/bin/bash

# Функция для вывода сообщений с префиксом
log() {
    echo "[Installer] $1"
}

# Проверка наличия sudo
check_sudo() {
    if [ "$EUID" -ne 0 ]; then
        return 1
    fi
    return 0
}

# Получение оригинального пользователя
get_original_user() {
    if [ -n "$SUDO_USER" ]; then
        echo "$SUDO_USER"
    else
        echo "$USER"
    fi
}

# Проверка и обновление pip
setup_pip() {
    local user=$(get_original_user)
    export PATH="/home/$user/.local/bin:$PATH"
    
    if ! command -v pip &> /dev/null; then
        log "Устанавливаю pip для пользователя $user..."
        sudo -u "$user" python3 -m ensurepip --user --upgrade
    fi
    
    log "Обновляю pip до последней версии..."
    sudo -u "$user" python3 -m pip install --user --upgrade pip
    log "Текущая версия pip: $(sudo -u "$user" pip --version)"
}

# Обновление списка пакетов apt
update_apt() {
    if check_sudo; then
        log "Обновляю списки пакетов apt..."
        apt-get update -qq
    else
        log "Пропускаю обновление apt (требуются права root)"
    fi
}

# Установка pip-зависимостей
install_pip_deps() {
    local dir="$1"
    local user=$(get_original_user)
    
    if [ -f "$dir/pip_requirements.txt" ]; then
        log "Устанавливаю pip-зависимости из $dir..."
        temp_log=$(mktemp)
        
        if sudo -u "$user" pip install --user -r "$dir/pip_requirements.txt" > "$temp_log" 2>&1; then
            log "УСПЕХ: pip-зависимости установлены"
            grep '^Successfully installed' "$temp_log" | sed 's/Successfully installed/[Installer]   - /'
        else
            log "ОШИБКА: Не удалось установить pip-зависимости"
            grep -E 'ERROR:|Could not install' "$temp_log" | sed 's/^/[Installer]   - /'
        fi
        
        rm -f "$temp_log"
    fi
}

# Установка apt-зависимостей
install_apt_deps() {
    local dir="$1"
    
    if [ -f "$dir/apt_requirements.txt" ]; then
        if ! check_sudo; then
            log "Пропускаю apt-зависимости (требуются права root)"
            return
        fi
        
        log "Устанавливаю apt-зависимости из $dir..."
        temp_log=$(mktemp)
        
        # Установка с обработкой ошибок
        while read -r pkg; do
            [[ -z "$pkg" || "$pkg" == \#* ]] && continue
            log "Устанавливаю $pkg..."
            if apt-get install -y "$pkg" >> "$temp_log" 2>&1; then
                log "  Успешно"
            else
                log "  ОШИБКА: не удалось установить $pkg"
            fi
        done < "$dir/apt_requirements.txt"
        
        # Проверка результатов
        if grep -q 'E:' "$temp_log"; then
            log "Проблемы при установке:"
            grep -E 'E:|W:' "$temp_log" | sed 's/^/[Installer]   - /'
        fi
        
        rm -f "$temp_log"
    fi
}

# Установка unitree_sdk2_python
install_unitree_sdk() {
    local user=$(get_original_user)
    
    if [ -d "./unitree_sdk2_python" ]; then
        log "Устанавливаю unitree_sdk2_python..."
        if sudo -u "$user" pip install --user ./unitree_sdk2_python; then
            log "Unitree SDK успешно установлен"
        else
            log "ОШИБКА: Не удалось установить Unitree SDK"
        fi
    fi
}

# Главная функция
main() {
    log "=== Начало установки зависимостей ==="
    
    # 1. Обновление системы
    update_apt
    
    # 2. Настройка pip
    setup_pip
    
    # 3. Установка зависимостей
    for dir in */; do
        dir=${dir%/}
        log "Обработка директории $dir..."
        
        install_pip_deps "$dir"
        install_apt_deps "$dir"
    done
    
    # 4. Установка Unitree SDK
    install_unitree_sdk
    
    log "=== Установка завершена ==="
    
    # Финал
    echo ""
    log "Решение проблем, если они возникли:"
    log "1. Для доступа к pip может потребоваться:"
    log "   export PATH=\"\$HOME/.local/bin:\$PATH\""
    log "2. Проверить установленные пакеты: pip list"
    log "3. При проблемах с apt-пакетами проверьте:"
    log "   cat <директория>/apt_requirements.txt"
}

main "$@"