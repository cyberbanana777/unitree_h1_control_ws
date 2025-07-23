#!/bin/bash

# Функция для вывода сообщений с префиксом
log() {
    echo "[Installer] $1"
}

# Проверка и обновление pip
check_and_update_pip() {
    # Проверка наличия pip
    if ! command -v pip &> /dev/null; then
        log "ОШИБКА: pip не установлен или не добавлен в PATH"
        log "Попробуйте установить pip командой:"
        log "  sudo apt-get install python3-pip  # Для Ubuntu/Debian"
        log "  или"
        log "  python -m ensurepip --upgrade      # Альтернативный способ"
        exit 1
    fi
    
    log "Обнаружен pip: $(pip --version)"
    
    # Обновление pip до последней версии
    log "Проверяем наличие обновлений для pip..."
    temp_log=$(mktemp)
    pip install --upgrade pip > "$temp_log" 2>&1
    
    if grep -q "Successfully installed pip" "$temp_log"; then
        new_version=$(grep "Successfully installed pip" "$temp_log" | awk '{print $4}')
        log "УСПЕХ: pip обновлён до версии $new_version"
    elif grep -q "Requirement already satisfied" "$temp_log"; then
        log "pip уже актуальной версии"
    else
        log "ПРЕДУПРЕЖДЕНИЕ: Не удалось обновить pip"
        grep -E 'ERROR:|Warning:' "$temp_log" | sed 's/^/[Installer]   - /'
    fi
    
    rm -f "$temp_log"
    log "Текущая версия pip: $(pip --version)"
}

# Установка unitree_sdk2_python
install_unitree_sdk() {
    if [ -d "./unitree_sdk2_python" ]; then
        log "Начинаю установку unitree_sdk2_python..."
        pip install ./unitree_sdk2_python || {
            log "ОШИБКА: Не удалось установить unitree_sdk2_python"
            exit 1
        }
    else
        log "Пропускаем установку unitree_sdk2_python - директория не найдена"
    fi
}

# Основная логика установки requirements.txt
process_requirements() {
    local found=0
    local not_found=0
    local found_dirs=()
    local not_found_dirs=()

    for dir in */; do
        dir=${dir%/}
        
        if [ -f "$dir/requirements.txt" ]; then
            ((found++))
            found_dirs+=("$dir")
            log "Найден requirements.txt в $dir. Начинаю установку зависимостей..."
            
            temp_log=$(mktemp)
            
            pip install -r "$dir/requirements.txt" > "$temp_log" 2>&1
            status=$?
            
            if [ $status -eq 0 ]; then
                log "УСПЕХ: Все зависимости из $dir установлены успешно"
                grep '^Successfully installed' "$temp_log" | sed 's/Successfully installed/[Installer]   - /'
            else
                log "ОШИБКА: Проблемы при установке зависимостей из $dir"
                grep -E 'ERROR:|Could not install' "$temp_log" | sed 's/^/[Installer]   - /'
                
                if grep -q '^Successfully installed' "$temp_log"; then
                    log "Часть пакетов установлена успешно:"
                    grep '^Successfully installed' "$temp_log" | sed 's/Successfully installed/[Installer]   - /'
                fi
            fi
            
            rm -f "$temp_log"
        else
            ((not_found++))
            not_found_dirs+=("$dir")
            log "Пропускаем $dir - requirements.txt не найден"
        fi
    done

    log ""
    log "===== ИТОГОВАЯ СТАТИСТИКА ====="
    log "Найдено requirements.txt в ${found} папках:"
    for d in "${found_dirs[@]}"; do
        log "  - $d"
    done

    log ""
    log "Не найдено requirements.txt в ${not_found} папках:"
    for d in "${not_found_dirs[@]}"; do
        log "  - $d"
    done
}

# Главная функция
main() {
    check_and_update_pip
    install_unitree_sdk
    process_requirements
    log "Процесс установки завершен"
}

main "$@"