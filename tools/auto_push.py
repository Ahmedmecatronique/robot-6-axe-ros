#!/usr/bin/env python3
"""Auto-push Git : ajoute, commit et pousse automatiquement les changements du projet vers GitHub.

Ne dépend QUE de la bibliothèque standard (subprocess) : il faut juste avoir Git installé, configuré
(`git config user.name` / `user.email`), et un remote `origin` déjà relié à GitHub avec des identifiants
en place (ex. `gh auth login`, ou un gestionnaire d'identifiants Windows/macOS).

Usage :
    python tools/auto_push.py                    # une fois : add + commit + push
    python tools/auto_push.py -m "mon message"    # message de commit personnalisé
    python tools/auto_push.py --dry-run           # affiche ce qui serait fait, sans rien exécuter
    python tools/auto_push.py --no-pull           # ne tire pas les changements distants avant de pousser
    python tools/auto_push.py --watch 60          # surveille en continu (vérifie toutes les 60 s), Ctrl+C pour arrêter
    python tools/auto_push.py --repo "C:\\chemin"  # exécute sur un autre dépôt que celui du script

Par défaut, ce script agit sur le dépôt qui contient ce fichier (le dossier parent de tools/).
"""
from __future__ import annotations

import argparse
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path


class GitError(RuntimeError):
    """Une commande git a échoué ; le message contient déjà stdout/stderr utiles."""


def run(args: list[str], cwd: Path, check: bool = True) -> subprocess.CompletedProcess:
    """Exécute une commande git et retourne le résultat (stdout/stderr capturés, texte)."""
    result = subprocess.run(args, cwd=cwd, capture_output=True, text=True)
    if check and result.returncode != 0:
        raise GitError(
            f"Échec de « {' '.join(args)} » (code {result.returncode})\n"
            f"{result.stdout}{result.stderr}".strip())
    return result


def log(msg: str) -> None:
    print(f'[{datetime.now().strftime("%H:%M:%S")}] {msg}')


# ------------------------------------------------------------------------------------------------
#  Étapes
# ------------------------------------------------------------------------------------------------
def ensure_repo(repo: Path) -> None:
    r = run(['git', 'rev-parse', '--is-inside-work-tree'], repo, check=False)
    if r.returncode != 0 or r.stdout.strip() != 'true':
        raise GitError(
            f"« {repo} » n'est pas un dépôt Git.\n"
            "Initialise-le d'abord : git init, git remote add origin <URL>, un premier commit.")


def has_local_changes(repo: Path) -> bool:
    return bool(run(['git', 'status', '--porcelain'], repo).stdout.strip())


def is_mid_operation(repo: Path) -> str | None:
    """Retourne un message si un merge/rebase/cherry-pick est en cours (dangereux d'automatiser dessus)."""
    git_dir = run(['git', 'rev-parse', '--git-dir'], repo).stdout.strip()
    base = repo / git_dir
    for name, label in (('MERGE_HEAD', 'une fusion (merge)'), ('REBASE_HEAD', 'un rebase'),
                        ('CHERRY_PICK_HEAD', 'un cherry-pick')):
        if (base / name).exists():
            return label
    return None


def current_branch(repo: Path) -> str:
    # symbolic-ref fonctionne même sur une branche "non née" (aucun commit encore) ; rev-parse --abbrev-ref non.
    r = run(['git', 'symbolic-ref', '-q', '--short', 'HEAD'], repo, check=False)
    if r.returncode != 0 or not r.stdout.strip():
        raise GitError("HEAD détachée (pas sur une branche) : bascule sur une branche avant de pousser.")
    return r.stdout.strip()


def has_upstream(repo: Path, branch: str) -> bool:
    r = run(['git', 'rev-parse', '--abbrev-ref', f'{branch}@{{upstream}}'], repo, check=False)
    return r.returncode == 0


def pull_rebase(repo: Path, branch: str, dry_run: bool) -> None:
    if not has_upstream(repo, branch):
        log('Pas encore de branche amont : rien à tirer pour le moment.')
        return
    if dry_run:
        log('[dry-run] git pull --rebase --autostash')
        return
    log('Récupération des changements distants (pull --rebase)…')
    r = run(['git', 'pull', '--rebase', '--autostash'], repo, check=False)
    if r.returncode != 0:
        raise GitError(
            'Le pull --rebase a échoué (conflit probable). Résous-le manuellement :\n'
            f'{r.stdout}{r.stderr}'.strip() + f'\n(dans : {repo})')


def stage_all(repo: Path, dry_run: bool) -> None:
    if dry_run:
        log('[dry-run] git add -A')
        return
    run(['git', 'add', '-A'], repo)


def commit(repo: Path, message: str, dry_run: bool) -> None:
    if dry_run:
        log(f'[dry-run] git commit -m "{message}"')
        return
    run(['git', 'commit', '-m', message], repo)
    log(f'Commit créé : {message}')


def commits_ahead(repo: Path, branch: str) -> int:
    """Nombre de commits locaux non encore poussés (suppose qu'une branche amont existe)."""
    return int(run(['git', 'rev-list', '--count', f'{branch}@{{upstream}}..HEAD'], repo).stdout.strip())


def push(repo: Path, branch: str, dry_run: bool) -> None:
    if dry_run:
        log(f'[dry-run] git push (branche {branch})')
        return
    if has_upstream(repo, branch):
        run(['git', 'push'], repo)
    else:
        log(f"Pas de branche amont pour « {branch} » : création avec « git push -u origin {branch} ».")
        run(['git', 'push', '-u', 'origin', branch], repo)
    log('Poussé vers GitHub avec succès.')


def default_message(repo: Path) -> str:
    n = len(run(['git', 'status', '--porcelain'], repo).stdout.strip().splitlines())
    plur = 's' if n > 1 else ''
    return f'Auto-push : {datetime.now().strftime("%Y-%m-%d %H:%M:%S")} ({n} fichier{plur} modifié{plur})'


# ------------------------------------------------------------------------------------------------
#  Un cycle complet
# ------------------------------------------------------------------------------------------------
def run_once(repo: Path, message: str | None, dry_run: bool, do_pull: bool) -> bool:
    """Exécute un cycle add+commit+push. Retourne True si quelque chose a été poussé.

    Pousse aussi bien les changements non validés (add+commit) que des commits déjà faits à la main
    et pas encore poussés (ex. juste après un premier commit initial) : on ne se contente pas de
    vérifier l'arbre de travail, on vérifie aussi si HEAD est en avance sur la branche amont.
    """
    ensure_repo(repo)
    blocked = is_mid_operation(repo)
    if blocked:
        raise GitError(f'{blocked.capitalize()} est en cours dans ce dépôt : résous-le avant de continuer.')

    branch = current_branch(repo)

    if do_pull:
        pull_rebase(repo, branch, dry_run)

    if has_local_changes(repo):
        msg = message or default_message(repo)
        stage_all(repo, dry_run)
        commit(repo, msg, dry_run)
    else:
        log('Aucun changement local à valider.')

    if not has_upstream(repo, branch):
        push(repo, branch, dry_run)          # premier push : crée la branche amont
        return True
    if dry_run or commits_ahead(repo, branch) > 0:
        push(repo, branch, dry_run)
        return True
    log('Rien à pousser (déjà synchronisé avec origin).')
    return False


# ------------------------------------------------------------------------------------------------
#  CLI
# ------------------------------------------------------------------------------------------------
def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('-m', '--message', default=None,
                    help="message de commit (par défaut : généré automatiquement avec la date et l'heure)")
    ap.add_argument('--repo', default=None,
                    help='dépôt cible (par défaut : le dossier parent de tools/, donc la racine du projet)')
    ap.add_argument('--dry-run', action='store_true', help="affiche les commandes sans les exécuter")
    ap.add_argument('--no-pull', action='store_true',
                    help='ne pas récupérer les changements distants avant de pousser')
    ap.add_argument('--watch', type=float, default=None, metavar='SECONDES',
                    help="surveille le dépôt en continu et pousse dès qu'il y a des changements "
                         "(vérification toutes les SECONDES) ; Ctrl+C pour arrêter")
    args = ap.parse_args(argv)

    repo = Path(args.repo).resolve() if args.repo else Path(__file__).resolve().parent.parent

    try:
        if args.watch is None:
            run_once(repo, args.message, args.dry_run, do_pull=not args.no_pull)
            return 0

        if args.watch <= 0:
            print('--watch attend une durée strictement positive (en secondes).', file=sys.stderr)
            return 2
        log(f'Surveillance de « {repo} » toutes les {args.watch:.0f} s (Ctrl+C pour arrêter)…')
        while True:
            try:
                run_once(repo, args.message, args.dry_run, do_pull=not args.no_pull)
            except GitError as exc:
                # en mode surveillance, une erreur ponctuelle (ex. conflit réseau) ne doit pas arrêter la boucle
                log(f'Erreur (on continue à surveiller) : {exc}')
            time.sleep(args.watch)

    except KeyboardInterrupt:
        log('Arrêt demandé (Ctrl+C).')
        return 0
    except GitError as exc:
        print(f'\n[ERREUR] {exc}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
