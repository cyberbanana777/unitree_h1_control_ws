#!/bin/bash

# Функция для вывода сообщений с префиксом
log() {
    echo "[Installer] $1"
}

# Глобальные переменные для статистики
declare -a found_pip_dirs not_found_pip_dirs
declare -a found_apt_dirs not_found_apt_dirs
declare -a installed_pip_pkgs failed_pip_pkgs
declare -a installed_apt_pkgs failed_apt_pkgs

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
        found_pip_dirs+=("$dir")
        log "Найден pip_requirements.txt в $dir"
        log "Устанавливаю pip-зависимости..."
        temp_log=$(mktemp)

        # Читаем все пакеты из файла
        while read -r pkg; do
            [[ -z "$pkg" || "$pkg" == \#* ]] && continue
            all_pip_pkgs+=("$pkg")
        done < "$dir/pip_requirements.txt"

        if sudo -u "$user" pip install --user -r "$dir/pip_requirements.txt" > "$temp_log" 2>&1; then
            log "УСПЕХ: pip-зависимости установлены"
            # Получаем список действительно установленных пакетов
            while read -r line; do
                pkg=$(echo "$line" | awk '{print $3}')
                installed_pip_pkgs+=("$pkg")
            done < <(grep '^Successfully installed' "$temp_log")

            # Находим неудачные установки (если есть)
            for pkg in "${all_pip_pkgs[@]}"; do
                if ! printf '%s\n' "${installed_pip_pkgs[@]}" | grep -qx "$(echo "$pkg" | cut -d'=' -f1 | cut -d'>' -f1 | cut -d'<' -f1)"; then
                    failed_pip_pkgs+=("$pkg")
                fi
            done

            grep '^Successfully installed' "$temp_log" | sed 's/Successfully installed/[Installer]   - /'
        else
            log "ОШИБКА: Не удалось установить pip-зависимости"
            # Сохраняем все пакеты как неудачные
            failed_pip_pkgs+=("${all_pip_pkgs[@]}")
            grep -E 'ERROR:|Could not install' "$temp_log" | sed 's/^/[Installer]   - /'
        fi

        rm -f "$temp_log"
    else
        not_found_pip_dirs+=("$dir")
    fi
}

# Установка apt-зависимостей
install_apt_deps() {
    local dir="$1"

    if [ -f "$dir/apt_requirements.txt" ]; then
        found_apt_dirs+=("$dir")

        if ! check_sudo; then
            log "Пропускаю apt-зависимости (требуются права root)"
            return
        fi

        log "Найден apt_requirements.txt в $dir"
        log "Устанавливаю apt-зависимости..."
        temp_log=$(mktemp)

        # Установка с обработкой ошибок
        while read -r pkg; do
            [[ -z "$pkg" || "$pkg" == \#* ]] && continue
            log "Устанавливаю $pkg..."
            if apt-get install -y "$pkg" >> "$temp_log" 2>&1; then
                installed_apt_pkgs+=("$pkg")
                log "  Успешно"
            else
                failed_apt_pkgs+=("$pkg")
                log "  ОШИБКА: не удалось установить $pkg"
            fi
        done < "$dir/apt_requirements.txt"

        # Проверка результатов
        if grep -q 'E:' "$temp_log"; then
            log "Проблемы при установке:"
            grep -E 'E:|W:' "$temp_log" | sed 's/^/[Installer]   - /'
        fi

        rm -f "$temp_log"
    else
        not_found_apt_dirs+=("$dir")
    fi
}

# Установка unitree_sdk2_python
install_unitree_sdk() {
    local user=$(get_original_user)

    if [ -d "./unitree_sdk2_python" ]; then
        log "Устанавливаю unitree_sdk2_python..."
        if sudo -u "$user" pip install --user -e ./unitree_sdk2_python; then
            log "Unitree SDK успешно установлен"
            installed_pip_pkgs+=("unitree_sdk2_python")
        else
            log "ОШИБКА: Не удалось установить Unitree SDK"
            failed_pip_pkgs+=("unitree_sdk2_python")
        fi
    fi
}

# Вывод подробного отчета
print_report() {
    echo ""
    log "===== ДЕТАЛЬНЫЙ ОТЧЕТ ====="

    # Отчет по pip
    log ""
    log "PIP ЗАВИСИМОСТИ:"
    log "Найдены requirements в директориях (${#found_pip_dirs[@]}):"
    for dir in "${found_pip_dirs[@]}"; do
        log "  - $dir/pip_requirements.txt"
    done

    log "Не найдены requirements в директориях (${#not_found_pip_dirs[@]}):"
    for dir in "${not_found_pip_dirs[@]}"; do
        log "  - $dir"
    done

    log ""
    log "Установленные pip-пакеты (${#installed_pip_pkgs[@]}):"
    for pkg in "${installed_pip_pkgs[@]}"; do
        log "  - $pkg"
    done

    log ""
    log "Неудачные установки pip (${#failed_pip_pkgs[@]}):"
    for pkg in "${failed_pip_pkgs[@]}"; do
        log "  - $pkg"
    done

    # Отчет по apt
    log ""
    log "APT ЗАВИСИМОСТИ:"
    log "Найдены requirements в директориях (${#found_apt_dirs[@]}):"
    for dir in "${found_apt_dirs[@]}"; do
        log "  - $dir/apt_requirements.txt"
    done

    log "Не найдены requirements в директориях (${#not_found_apt_dirs[@]}):"
    for dir in "${not_found_apt_dirs[@]}"; do
        log "  - $dir"
    done

    log ""
    log "Установленные apt-пакеты (${#installed_apt_pkgs[@]}):"
    for pkg in "${installed_apt_pkgs[@]}"; do
        log "  - $pkg"
    done

    log ""
    log "Неудачные установки apt (${#failed_apt_pkgs[@]}):"
    for pkg in "${failed_apt_pkgs[@]}"; do
        log "  - $pkg"
    done

    log ""
    log "===== РЕКОМЕНДАЦИИ ====="
    if [ ${#failed_pip_pkgs[@]} -gt 0 ]; then
        log "Для решения проблем с pip:"
        log "1. Попробуйте установить вручную:"
        for pkg in "${failed_pip_pkgs[@]}"; do
            log "   pip install --user $pkg"
        done
        log "2. Проверьте подключение к интернету"
        log "3. Проверьте совместимость версий пакетов"
    fi

    if [ ${#failed_apt_pkgs[@]} -gt 0 ]; then
        log ""
        log "Для решения проблем с apt:"
        log "1. Попробуйте установить вручную:"
        for pkg in "${failed_apt_pkgs[@]}"; do
            log "   sudo apt-get install $pkg"
        done
        log "2. Проверьте доступность пакетов:"
        log "   sudo apt-cache policy <имя_пакета>"
        log "3. Попробуйте обновить кэш: sudo apt-get update"
    fi

    if [ ${#failed_pip_pkgs[@]} -eq 0 ] && [ ${#failed_apt_pkgs[@]} -eq 0 ]; then
        log "Все зависимости успешно установлены!"
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

    # 5. Вывод отчета
    print_report

    log "=== Установка завершена ==="
}

main "$@"
