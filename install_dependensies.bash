#!/bin/bash

# Установщик зависимостей в виде функции
# Использование: source install-deps.bash && main

# Цвета для вывода
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
MAGENTA='\033[0;35m'
NC='\033[0m' # No Color

# Функция для цветного вывода сообщений
log() {
    echo -e "[Installer] $1"
}

log_color() {
    local color=$1
    shift
    echo -e "${color}[Installer] $*${NC}"
}

log_success() { log_color "$GREEN" "$@"; }
log_error() { log_color "$RED" "$@"; }
log_warning() { log_color "$YELLOW" "$@"; }
log_info() { log_color "$BLUE" "$@"; }
log_debug() { log_color "$CYAN" "$@"; }

# Функция для вывода отсортированного списка пакетов
log_sorted() {
    local prefix="$1"
    local color="$2"
    shift 2
    local packages=("$@")
    
    # Сортируем массив
    IFS=$'\n' sorted_packages=($(sort <<<"${packages[*]}"))
    unset IFS
    
    for pkg in "${sorted_packages[@]}"; do
        log_color "$color" "$prefix$pkg"
    done
}

# Функция для обновления pip
upgrade_pip() {
    local user=$(get_original_user)
    export PATH="/home/$user/.local/bin:$PATH"
    
    log_info "Проверяю версию pip..."
    
    if ! command -v pip &> /dev/null; then
        log_warning "pip не установлен, пропускаю обновление"
        return 1
    fi
    
    # Получаем текущую версию pip
    local current_version=$(sudo -u "$user" pip --version 2>/dev/null | awk '{print $2}')
    
    if [ -z "$current_version" ]; then
        log_warning "Не удалось определить версию pip"
        return 1
    fi
    
    log_info "Текущая версия pip: $current_version"
    log_info "Обновляю pip до последней версии..."
    
    # Обновляем pip
    if sudo -u "$user" pip install --user --upgrade pip; then
        local new_version=$(sudo -u "$user" pip --version 2>/dev/null | awk '{print $2}')
        log_success "pip успешно обновлен: $current_version → $new_version"
        return 0
    else
        log_error "Ошибка при обновлении pip"
        return 1
    fi
}

# Выполнение addition.bash если он существует
execute_addition() {
    local addition_file="install_additions.bash"
    
    if [ -f "$addition_file" ]; then
        log_info "Найден $addition_file, выполнение..."
        log_info "=== Начало выполнения $addition_file ==="
        
        if bash "$addition_file"; then
            log_success "=== $addition_file успешно выполнен ==="
            return 0
        else
            local exit_code=$?
            log_error "=== $addition_file завершился с ошибкой (код: $exit_code) ==="
            return $exit_code
        fi
    else
        log_debug "Файл $addition_file не найден, пропускаю"
        return 0
    fi
}

# Функция для удаления дубликатов из массива
remove_duplicates() {
    local -n arr=$1
    local -A seen
    local -a unique
    for i in "${arr[@]}"; do
        if [[ -z "${seen[$i]}" ]]; then
            unique+=("$i")
            seen[$i]=1
        fi
    done
    arr=("${unique[@]}")
}

# Функция для добавления уникальных pip пакетов
add_unique_pip_package() {
    local package="$1"
    if [[ -z "${all_pip_packages_hash[$package]}" ]]; then
        all_pip_packages_hash[$package]=1
        log_debug "  Добавлен pip-пакет: $package"
    else
        log_debug "  Пропуск дубликата pip: $package"
    fi
}

# Функция для добавления уникальных apt пакетов
add_unique_apt_package() {
    local package="$1"
    if [[ -z "${all_apt_packages_hash[$package]}" ]]; then
        all_apt_packages_hash[$package]=1
        log_debug "  Добавлен apt-пакет: $package"
    else
        log_debug "  Пропуск дубликата apt: $package"
    fi
}

# Получение оригинального пользователя
get_original_user() {
    if [ -n "$SUDO_USER" ]; then
        echo "$SUDO_USER"
    else
        echo "$USER"
    fi
}

# Проверка наличия sudo
check_sudo() {
    if [ "$EUID" -ne 0 ]; then
        return 1
    fi
    return 0
}

# Проверка установленных pip пакетов
get_installed_pip_packages() {
    local user=$(get_original_user)
    export PATH="/home/$user/.local/bin:$PATH"
    
    if command -v pip &> /dev/null; then
        # Получаем список установленных пакетов в формате "пакет==версия"
        sudo -u "$user" pip list --format=freeze 2>/dev/null | while read -r line; do
            echo "$line"
        done
    fi
}

# Проверка установленных apt пакетов
get_installed_apt_packages() {
    # Получаем список установленных пакетов
    if command -v dpkg-query &> /dev/null; then
        dpkg-query -W -f='${Package}\n' 2>/dev/null
    elif command -v apt &> /dev/null; then
        apt list --installed 2>/dev/null | grep -v 'Listing...' | cut -d'/' -f1
    fi
}

# Проверка доступности apt пакетов
check_apt_package_available() {
    local package="$1"
    
    if command -v apt-cache &> /dev/null; then
        apt-cache show "$package" >/dev/null 2>&1
        return $?
    else
        # Если apt-cache недоступен, предполагаем что пакет доступен
        return 0
    fi
}

# Проверка, установлен ли pip пакет
check_pip_package_installed() {
    local package="$1"
    local installed_packages="$2"
    
    # Извлекаем имя пакета без версии/условий
    local package_name=$(echo "$package" | cut -d'=' -f1 | cut -d'>' -f1 | cut -d'<' -f1 | cut -d'[' -f1)
    
    # Проверяем, есть ли пакет в списке установленных
    echo "$installed_packages" | grep -i "^${package_name}==" >/dev/null 2>&1
    return $?
}

# Проверка, установлен ли apt пакет
check_apt_package_installed() {
    local package="$1"
    local installed_packages="$2"
    
    # Проверяем, есть ли пакет в списке установленных
    echo "$installed_packages" | grep -i "^${package}$" >/dev/null 2>&1
    return $?
}

# Обновление списка пакетов apt
update_apt() {
    if check_sudo; then
        log_info "Обновляю списки пакетов apt..."
        apt-get update -qq
    else
        log_warning "Пропускаю обновление apt (требуются права root)"
    fi
}

# Установка pip пакетов
install_pip_packages() {
    local packages=("$@")
    local user=$(get_original_user)
    
    if [ ${#packages[@]} -eq 0 ]; then
        log_success "Нет pip пакетов для установки"
        return 0
    fi
    
    log_info "Устанавливаю pip пакеты: ${#packages[@]} шт."
    export PATH="/home/$user/.local/bin:$PATH"
    
    # Собираем все пакеты в одну строку
    local packages_str=$(printf " %s" "${packages[@]}")
    packages_str=${packages_str:1}
    
    log_debug "Команда: pip install --user $packages_str"
    
    if sudo -u "$user" pip install --user $packages_str; then
        log_success "PIP пакеты успешно установлены"
        return 0
    else
        log_error "Ошибка установки PIP пакетов"
        return 1
    fi
}

# Установка apt пакетов
install_apt_packages() {
    local packages=("$@")
    
    if [ ${#packages[@]} -eq 0 ]; then
        log_success "Нет apt пакетов для установки"
        return 0
    fi
    
    if ! check_sudo; then
        log_error "Для установки apt пакетов требуются права root!"
        log_info "Запустите скрипт с sudo для установки apt пакетов"
        return 1
    fi
    
    log_info "Устанавливаю apt пакеты: ${#packages[@]} шт."
    
    # Собираем все пакеты в одну строку
    local packages_str=$(printf " %s" "${packages[@]}")
    packages_str=${packages_str:1}
    
    log_debug "Команда: apt-get install -y $packages_str"
    
    if apt-get install -y $packages_str; then
        log_success "APT пакеты успешно установлены"
        return 0
    else
        log_error "Ошибка установки APT пакетов"
        return 1
    fi
}

# Сбор pip-зависимостей
collect_pip_deps() {
    local dir="$1"
    
    if [ -f "$dir/pip_requirements.txt" ]; then
        found_pip_dirs+=("$dir")
        log_info "Найден pip_requirements.txt в $dir"
        
        # Читаем все пакеты из файла
        while read -r pkg; do
            [[ -z "$pkg" || "$pkg" == \#* ]] && continue
            add_unique_pip_package "$pkg"
        done < "$dir/pip_requirements.txt"
    else
        not_found_pip_dirs+=("$dir")
    fi
}

# Сбор apt-зависимостей
collect_apt_deps() {
    local dir="$1"
    
    if [ -f "$dir/apt_requirements.txt" ] || [ -f "$dir/apt-get_requirements.txt" ]; then
        local apt_file=""
        if [ -f "$dir/apt_requirements.txt" ]; then
            apt_file="$dir/apt_requirements.txt"
        else
            apt_file="$dir/apt-get_requirements.txt"
        fi
        
        found_apt_dirs+=("$dir")
        log_info "Найден $apt_file в $dir"
        
        # Читаем все пакеты из файла
        while read -r pkg; do
            [[ -z "$pkg" || "$pkg" == \#* ]] && continue
            add_unique_apt_package "$pkg"
        done < "$apt_file"
    else
        not_found_apt_dirs+=("$dir")
    fi
}

# Анализ зависимостей
analyze_dependencies() {
    local installed_pip="$1"
    local installed_apt="$2"
    
    log_info "Анализ pip-зависимостей..."
    for pkg in "${!all_pip_packages_hash[@]}"; do
        if check_pip_package_installed "$pkg" "$installed_pip"; then
            installed_pip_packages+=("$pkg")
        else
            missing_pip_packages+=("$pkg")
        fi
    done
    
    # Удаляем дубликаты из массивов (на всякий случай)
    remove_duplicates installed_pip_packages
    remove_duplicates missing_pip_packages
    
    log_info "Анализ apt-зависимостей..."
    for pkg in "${!all_apt_packages_hash[@]}"; do
        if check_apt_package_installed "$pkg" "$installed_apt"; then
            installed_apt_packages+=("$pkg")
        else
            missing_apt_packages+=("$pkg")
        fi
    done
    
    # Удаляем дубликаты из массивов (на всякий случай)
    remove_duplicates installed_apt_packages
    remove_duplicates missing_apt_packages
}

# Установка недостающих пакетов
install_missing_packages() {
    local pip_to_install=() apt_to_install=()
    
    # Копируем массивы для установки
    pip_to_install=("${missing_pip_packages[@]}")
    apt_to_install=("${missing_apt_packages[@]}")
    
    if [ ${#pip_to_install[@]} -eq 0 ] && [ ${#apt_to_install[@]} -eq 0 ]; then
        log_success "Все пакеты уже установлены!"
        return 0
    fi
    
    log_info "=== Начало установки пакетов ==="
    
    # Сначала обновляем pip (важно делать это ДО установки пакетов)
    upgrade_pip
    
    # Установка pip пакетов
    if [ ${#pip_to_install[@]} -gt 0 ]; then
        log_info "Установка PIP пакетов..."
        install_pip_packages "${pip_to_install[@]}"
    fi
    
    # Установка apt пакетов
    if [ ${#apt_to_install[@]} -gt 0 ]; then
        log_info "Установка APT пакетов..."
        install_apt_packages "${apt_to_install[@]}"
    fi
    
    log_success "=== Установка завершена ==="
}

# Вывод отчета о собранных зависимостях
print_dependencies_report() {
    echo ""
    log_info "===== ОТЧЕТ О СОБРАННЫХ ЗАВИСИМОСТЯХ ====="
    
    # Получаем массивы из хэшей
    local all_pip_packages=("${!all_pip_packages_hash[@]}")
    local all_apt_packages=("${!all_apt_packages_hash[@]}")
    
    # Отчет по pip
    echo ""
    log_info "PIP ЗАВИСИМОСТИ:"
    log "Найдены requirements в директориях (${#found_pip_dirs[@]}):"
    for dir in "${found_pip_dirs[@]}"; do
        log "  - $dir/pip_requirements.txt"
    done
    
    log "Не найдены requirements в директориях (${#not_found_pip_dirs[@]}):"
    for dir in "${not_found_pip_dirs[@]}"; do
        log "  - $dir"
    done
    
    echo ""
    log_info "ВСЕ УНИКАЛЬНЫЕ pip-пакеты (${#all_pip_packages[@]}):"
    log_sorted "  - " "$CYAN" "${all_pip_packages[@]}"
    
    echo ""
    log_success "УЖЕ УСТАНОВЛЕННЫЕ pip-пакеты (${#installed_pip_packages[@]}):"
    log_sorted "  - " "$GREEN" "${installed_pip_packages[@]}"
    
    echo ""
    if [ ${#missing_pip_packages[@]} -gt 0 ]; then
        log_error "ОТСУТСТВУЮЩИЕ pip-пакеты (${#missing_pip_packages[@]}):"
        log_sorted "  - " "$RED" "${missing_pip_packages[@]}"
    else
        log_success "ОТСУТСТВУЮЩИЕ pip-пакеты (0):"
        log_success "  - Все пакеты установлены!"
    fi
    
    # Отчет по apt
    echo ""
    log_info "APT ЗАВИСИМОСТИ:"
    log "Найдены requirements в директориях (${#found_apt_dirs[@]}):"
    for dir in "${found_apt_dirs[@]}"; do
        if [ -f "$dir/apt_requirements.txt" ]; then
            log "  - $dir/apt_requirements.txt"
        else
            log "  - $dir/apt-get_requirements.txt"
        fi
    done
    
    log "Не найдены requirements в директориях (${#not_found_apt_dirs[@]}):"
    for dir in "${not_found_apt_dirs[@]}"; do
        log "  - $dir"
    done
    
    echo ""
    log_info "ВСЕ УНИКАЛЬНЫЕ apt-пакеты (${#all_apt_packages[@]}):"
    log_sorted "  - " "$CYAN" "${all_apt_packages[@]}"
    
    echo ""
    log_success "УЖE УСТАНОВЛЕННЫЕ apt-пакеты (${#installed_apt_packages[@]}):"
    log_sorted "  - " "$GREEN" "${installed_apt_packages[@]}"
    
    echo ""
    if [ ${#missing_apt_packages[@]} -gt 0 ]; then
        log_error "ОТСУТСТВУЮЩИЕ apt-пакеты (${#missing_apt_packages[@]}):"
        log_sorted "  - " "$RED" "${missing_apt_packages[@]}"
    else
        log_success "ОТСУТСТВУЮЩИЕ apt-пакеты (0):"
        log_success "  - Все пакеты установлены!"
    fi
    
    # Статистика
    echo ""
    log_info "===== СТАТИСТИКА ====="
    log "PIP: ${#installed_pip_packages[@]}/${#all_pip_packages[@]} установлено, ${#missing_pip_packages[@]} требуется установить"
    log "APT: ${#installed_apt_packages[@]}/${#all_apt_packages[@]} установлено, ${#missing_apt_packages[@]} требуется установить"
}

# Главная функция
install_dependensies() {
    # Глобальные переменные для сбора зависимостей
    local found_pip_dirs=() not_found_pip_dirs=()
    local found_apt_dirs=() not_found_apt_dirs=()
    local -A all_pip_packages_hash all_apt_packages_hash
    local installed_pip_packages=() missing_pip_packages=()
    local installed_apt_packages=() missing_apt_packages=()
    
    log_info "=== Сбор информации о зависимостях ==="
    
    # Обновляем apt (если есть права)
    update_apt
    
    # Получаем списки установленных пакетов
    local installed_pip=$(get_installed_pip_packages)
    local installed_apt=$(get_installed_apt_packages)
    
    # Сбор зависимостей
    for dir in */; do
        dir=${dir%/}
        log_info "Обработка директории $dir..."
        
        collect_pip_deps "$dir"
        collect_apt_deps "$dir"
    done
    
    # Анализ зависимостей
    analyze_dependencies "$installed_pip" "$installed_apt"
    
    # Вывод собранных зависимостей
    print_dependencies_report
    
    # Установка недостающих пакетов (автоматически без подтверждения)
    install_missing_packages
    
    log_success "=== Процесс завершен ==="
}


main (){
    upgrade_pip

    install_dependensies
    # Выполняем install_additions.bash если он существует
    execute_addition
}

main "$@"

